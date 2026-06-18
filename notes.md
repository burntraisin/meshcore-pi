# Create a Virtual Environment

```bash
$ python -m venv venv
$ . ./venv/bin/activate
```

# Packages to Install

```bash
$ pip install pycryptodome aiotools pyserial_asyncio typing-extensions LoRaRF scapy==2.5.0
```

Note that on newer Pis, the `rpi.gpio` package is oudated and `rpi-lgpio` should be installed.

```bash
$ pip uninstall rpi.gpio
$ pip install rpi-lgpio
```

# Run MeshCore

```bash
$ ./meshcore.py config.toml
```