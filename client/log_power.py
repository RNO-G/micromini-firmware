import datetime
import subprocess
import re
import time

def get_hostname():
    sp = subprocess.run(["hostnamectl", "hostname"], capture_output=True)
    sp.check_returncode()
    return sp.stdout.decode("utf-8").strip('\n')
    

def measure():
    sp = subprocess.run(["./micromini-tool", "measure"])
    sp.check_returncode()

def read_measurement():
    sp = subprocess.run(["./micromini-tool", "read-sensor-measurement"], capture_output=True)
    sp.check_returncode()
    return sp.stdout

def read_ain():
    while True:
        sp = subprocess.run(["./micromini-tool", "get-ain-ready"], capture_output=True)

        m = re.match(b"get-ain-ready: ([0,1])\n", sp.stdout)
        if int(m.group(1)):
            sp = subprocess.run(["./micromini-tool", "get-ain-ready"], capture_output=True)
            sp.check_returncode()

            break

    return sp.stdout

def get_file_name(prefix="power_"):
    fname = prefix + datetime.datetime.now(datetime.UTC).strftime("%Y_%m_%d") + ".txt"
    return fname

def write_to_file(fname, data, separator="\n"):

    with open(fname, "a") as f:
        f.write(str(data))
        f.write(separator)

def parse_data(data):
    uptime_str, temperature_str, pv_str, wind_str, _ = data.split(b"\n")

    uptime_match = re.match(b'Measurement at uptime = ([0-9]*)', uptime_str)
    assert uptime_match is not None, "Parsing uptime failed"

    temperature_match = re.match(b'\t T_local = ([-+]?[0-9]*\.?[0-9]*)\t T1 = ([-+]?[0-9]*\.?[0-9]*)\t T2 = ([-+]?[0-9]*\.?[0-9]*)', temperature_str)
    assert temperature_match is not None, "Parsing temperature failed"

    pv_match = re.match(b'\t PV:  ([0-9]*\.?[0-9]*) V, ([0-9]*\.?[0-9]*) A', pv_str)
    assert pv_match is not None, "Parsing pv failed"

    wind_match = re.match(b'\t TURBINE:  ([0-9]*\.?[0-9]*) V, ([0-9]*\.?[0-9]*) A', wind_str)
    assert wind_match is not None, "Parsing wind failed"

    data = {
        "time": datetime.datetime.now(datetime.UTC).timestamp(),
        "uptime": int(uptime_match.group(1)),
        "t_local": float(temperature_match.group(1)),
        "t_1": float(temperature_match.group(2)),
        "t_2": float(temperature_match.group(3)),
        "pv_V": float(pv_match.group(1)),
        "pv_A": float(pv_match.group(2)),
        "wind_V": float(wind_match.group(1)),
        "wind_A": float(wind_match.group(2)),
    }

    return data


if __name__ == "__main__":

    time_interval_between_measurments = 20  # sec
    host = get_hostname()
    while True:
        fname = get_file_name(prefix=f"/data/power/{host}_power_")
        measure()
        time.sleep(1)

        data = read_measurement()
        data = parse_data(data)
        write_to_file(fname, data)

        time.sleep(time_interval_between_measurments)
