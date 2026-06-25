import asyncio
from aiotools import current_taskgroup
import threading
import time
from binascii import unhexlify, hexlify
from collections import deque

from .interface import Interface
from configuration import ConfigView, get_config

from LoRaRF import SX127x

import logging
logger = logging.getLogger(__name__)

# DIO3 TCXO control settings
# SetDio3AsTcxoCtrl
#    DIO3_OUTPUT_1_6                        = 0x00        # DIO3 voltage output for TCXO: 1.6 V
#    DIO3_OUTPUT_1_7                        = 0x01        #                               1.7 V
#    DIO3_OUTPUT_1_8                        = 0x02        #                               1.8 V
#    DIO3_OUTPUT_2_2                        = 0x03        #                               2.2 V
#    DIO3_OUTPUT_2_4                        = 0x04        #                               2.4 V
#    DIO3_OUTPUT_2_7                        = 0x05        #                               2.7 V
#    DIO3_OUTPUT_3_0                        = 0x06        #                               3.0 V
#    DIO3_OUTPUT_3_3                        = 0x07        #                               3.3 V
#    TCXO_DELAY_2_5                         = 0x0140      # TCXO delay time: 2.5 ms
#    TCXO_DELAY_5                           = 0x0280      #                  5 ms
#    TCXO_DELAY_10                          = 0x0560      #                  10 ms

DIO3_VOLTAGE = {
    1.6: 0x00,
    1.7: 0x01,
    1.8: 0x02,
    2.2: 0x03,
    2.4: 0x04,
    2.7: 0x05,
    3.0: 0x06,
    3.3: 0x07
}

TCXO_DELAY = {
    2.5: 0x0140,
    5: 0x0280,
    10: 0x0560
}

class SX127xInterface(Interface):
    """
    Communicate with a directly connected LoRa interface

    """
    def __init__(self, config:ConfigView):
        super().__init__() 
        self._name = "SX127x device interface"
        self._hw_lock = threading.Lock()

        # Flag to signal when data has been transmitted
        self.txdone = asyncio.Event()
        self.txtime = 0

        # Config for US MeshCore
        config.set_default(get_config({
            "frequency": 910525000, "sf": 7, "bw":62500, "cr":5,
            "txpower": 17, "airtime": 100,
            # AdaFruit Bonnet SX127x for Raspberry Pi
            "spi":0, "cs": 1, "irq": 22, "reset": 25
        }))

        self.freq = config.get("frequency")
        self.sf = config.get("sf")
        self.bw = config.get("bw")
        self.cr = config.get("cr")
        self.txpower = config.get("txpower")
        airtime = config.get("airtime", 100)

        spi = config.get("spi")
        cs = config.get("cs")
        irq = config.get("irq")
        reset = config.get("reset")
        txen = config.get("txen", -1)
        rxen = config.get("rxen", -1)

        self.LoRa = SX127x()

        if not self.LoRa.begin(spi, cs, reset, irq, txen, rxen):
            logger.error("LoRa interface did not start")
            raise ValueError("LoRa interface did not start")

        self.LoRa.setFrequency(self.freq)
        # self.LoRa.setTxPower(17, self.LoRa.TX_POWER_PA_BOOST)
        # self.LoRa.setTxPower(self.txpower, self.LoRa.TX_POWER_PA_BOOST) #self.LoRa.RX_GAIN_BOOSTED
        self.LoRa.setTxPower(13, self.LoRa.TX_POWER_RFO)
        self.LoRa.setRxGain(True, True)
        self.LoRa.setLoRaModulation(self.sf, self.bw, self.cr, False)

        self.LoRa.setLoRaPacket(self.LoRa.HEADER_EXPLICIT, 16, 255, True, False)
        self.LoRa.setSyncWord(0x12)

        self.airtime_dutycycle = airtime     # % duty cycle (default 10%)

        self.airtime_txtimestamp = deque([0,0,0,0,0], maxlen=5)
        self.airtime_txtime = deque([0,0,0,0,0], maxlen=5)

        logger.debug(f"Configired LoRa interface on SPI{spi}:{cs} for {self.freq/1000000:0.3f}MHz, BW: {self.bw/1000}KHz, SF: {self.sf}, CR: {self.cr}")

    # FIXME: This thread busywaits on data from the LoRa chip. This could be a setting I've missed,
    # or it might just be how the library works. Either way, it sits there using up an entire core.
    # Need either better config, a better library, or to rewrite the current one so it behaves nicely.
    def rx_thread(self):

        logger.debug("LoRa rx thread listening")
        s = ["STATUS_DEFAULT", "STATUS_TX_WAIT", "STATUS_TX_TIMEOUT", "STATUS_TX_DONE", "STATUS_RX_WAIT", "STATUS_RX_CONTINUOUS", "STATUS_RX_TIMEOUT", "STATUS_RX_DONE", "STATUS_HEADER_ERR", "STATUS_CRC_ERR", "STATUS_CAD_WAIT", "STATUS_CAD_DETECTED", "STATUS_CAD_DONE"]

        time.sleep(2)

        while True:
            with self._hw_lock:
                status = self.LoRa.status()
                logger.debug(f"Status: {s[status]}")

                if status == self.LoRa.STATUS_RX_DONE:
                    logger.debug(f"Packet received, {self.LoRa.available()} bytes")
                    data = bytearray()
                    while self.LoRa.available():
                        data.append(self.LoRa.read())

                    rssi = self.LoRa.packetRssi()
                    snr = self.LoRa.snr()

                    self.eventloop.call_soon_threadsafe(self.rx_q.put_nowait, (data, rssi, snr))
                    self.LoRa.writeRegister(self.LoRa.REG_IRQ_FLAGS, 0xFF) # Clear interrupts
                    logger.debug(f"Packet data, {hexlify(data).decode()}")
                    
                elif status == self.LoRa.STATUS_CRC_ERR or status == self.LoRa.STATUS_HEADER_ERR:
                    logger.info(f"RX packet error status: {s[status]}")
                    self.LoRa.writeRegister(self.LoRa.REG_IRQ_FLAGS, 0xFF)
                    
                elif status == self.LoRa.STATUS_TX_DONE:
                    self.eventloop.call_soon_threadsafe(self.tx_done, self.LoRa.transmitTime())
                    self.LoRa.writeRegister(self.LoRa.REG_IRQ_FLAGS, 0xFF)

    # FIXME race condition here - what is the proper timeout for a transmission?
    def tx_done(self, tx_time):
        self.txtime = tx_time
        self.txdone.set()

    def transmit_wait(self):
        # Based on the last 5 transmissions, are we within the duty cycle limit?
        tx_earliest = self.airtime_txtimestamp[0]

        # How long since the first transmission in the log?
        tx_period = time.time() - tx_earliest
        # Total time (ms)
        tx_total = sum(self.airtime_txtime)
        duty_cycle = 100*(tx_total/1000)/tx_period

        if tx_earliest > 0:
            # We have recorded 5 transmissions
            logger.debug(f"Duty cycle for last {len(self.airtime_txtimestamp)} transmissions: {duty_cycle:0.2f}%")

        # Sleep until the duty cycle would be less than 10% (or whatever airtime_dutycycle is)
        # Rather than wait until we hit the duty cycle limit and then sleep, if the duty cycle is half
        # the limit (eg, 5%), sleep for half the required time. If it's a quarter, sleep for 25% of the
        # required time. This will have the effect of spreading out the wait periods, rather than
        # transmitting a bunch of packets then a long pause
        for c in range(3):
            fraction = 1/(1<<c)     # 1/1, 1/2, 1/4
            airtime_dutycycle = self.airtime_dutycycle * fraction

            if duty_cycle > airtime_dutycycle:
                tx_min = (tx_earliest + (tx_total/1000)/(airtime_dutycycle / 100) - time.time()) * fraction

                if tx_min>0:
                    logger.debug(f"Sleep for {tx_min:0.2f} seconds for duty cycle compliance ({airtime_dutycycle}%)")
                    return tx_min

        return 0

    async def transmit(self, packetdata):
        logger.debug(f"Transmitting: {hexlify(packetdata).decode()}")

        payload_len = len(packetdata)
        logger.debug(f"The payload length is {payload_len}")
                
        self.txdone.clear()
        self.txtime = 0

        def _safe_tx():
            with self._hw_lock:
                self.LoRa.beginPacket()
                self.LoRa.put(packetdata)
                self.LoRa.endPacket()

        await self.eventloop.run_in_executor(None, _safe_tx)

        try:
            await asyncio.wait_for(self.txdone.wait(), 5)

            logger.debug("Transmit time: {0:0.2f} ms".format(self.txtime))

            self.airtime_txtimestamp.append(time.time())
            self.airtime_txtime.append(self.txtime)

        except TimeoutError:
            logger.debug("Transmit timed out")

        finally:
            # Back to receiving
            def _restore_rx():
                with self._hw_lock:
                    self.LoRa.request(self.LoRa.RX_CONTINUOUS)
            await self.eventloop.run_in_executor(None, _restore_rx)
    
        self.txdone.clear()
        return self.txtime

    # Return a tuple containing frequency (kHz), bandwidth (Hz), spreading factor, coding rate,
    # tx power (dBm), maximum tx power (dBm)
    def get_radioconfig(self):
        return (self.freq//1000, self.bw, self.sf, self.cr, self.txpower, 27)

    async def start(self):
        self.eventloop = asyncio.get_running_loop()
        # Start the receiver in its own thread as it's not asynchronous, make it a daemon thread so it
        # doesn't stop the program terminating

        self.rxthread = threading.Thread(target=self.rx_thread, daemon=True)
        self.rxthread.start()