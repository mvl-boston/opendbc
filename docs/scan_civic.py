#!/usr/bin/env python3

import json
from urllib.request import urlopen
from openpilot.tools.lib.logreader import LogReader

data = json.load(urlopen(
    "https://huggingface.co/datasets/commaai/commaCarSegments/raw/main/database.json"
))

for fp, routes in data.items():

    if "HONDA_CIVIC" not in fp:
        continue

    found = False

    for route in routes[:10]:

        base = "/".join(route.split("/")[:2])

        for seg in range(3):

            try:
                lr = LogReader(f"{base}/{seg}")
            except Exception:
                continue

            for msg in lr:

                if msg.which() != "can":
                    continue

                for frame in msg.can:

                    if frame.address == 0x1BE:
                        print(fp, f"{base}/{seg}")
                        found = True
                        break

                if found:
                    break

            if found:
                break

        if found:
            break
