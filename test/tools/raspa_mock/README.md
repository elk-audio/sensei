# raspa_mock

Mock of `raspa` for testing `sensei` interactions, sending simulated MCU input param data over the socket, and verifying that we receive the expected OSC messages from `sensei`. 

## Usage

You should first build `sensei`, which the `main.py` Python script expects to be found in `../../../build/sensei` (otherwise modify the `SENSEI_BINARY` path).

Then you'll need to:

- create a Python venv
- activate it
- install the requirements
- generate the `gpio_protocol` bindings
- and run the `main.py` script

As commands:
```
$ python3 -m venv venv
$ source venv/bin/activate
$ pip install -r requirements.txt
$ clang2py ../../../elk-gpio-protocol/include/gpio_protocol/gpio_protocol.h >gpio_protocol.py
$ python main.py
```

You can use `ctrl-C` to kill the running process. Once the basic script runs you can also modify the sensei config by changing `sensei_config.json` in this folder, and changing the values that are sent to `sensei` in the `main()` function inside `main.py`.

---
Copyright 2017-2025 Elk Audio AB, Stockholm, Sweden.
