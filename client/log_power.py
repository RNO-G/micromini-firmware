import datetime
import subprocess
import re
import time
import sys
import os

DIRECTORY = os.path.dirname(os.path.realpath(__file__))

def get_hostname():
    sp = subprocess.run(["hostnamectl", "hostname"], capture_output=True)
    sp.check_returncode()
    return sp.stdout.decode("utf-8").strip('\n')


def measure():
    sp = subprocess.run([f"{DIRECTORY}/micromini-tool", "measure"])
    sp.check_returncode()


def reset():
    # We experienced infrequently issues with the data in some stations
    # which were solved with running the resetting ...
    sp = subprocess.run([f"{DIRECTORY}/micromini-tool", "reset"])
    sp.check_returncode()


def read_measurement():
    sp = subprocess.run([f"{DIRECTORY}/micromini-tool", "read-sensor-measurement"], capture_output=True)
    sp.check_returncode()
    return sp.stdout

def read_rpm(set_threshold_value, set_sampling_rate):

    #should be at 0V (set to 2048 which should be half of the ADC range)
    set_th_rising = subprocess.run([f"{DIRECTORY}/micromini-tool", "set-ain-rising-threshold", set_threshold_value], capture_output=False)
    set_th_rising.check_returncode()
    set_th_falling = subprocess.run([f"{DIRECTORY}/micromini-tool", "set-ain-falling-threshold", set_threshold_value], capture_output=False)
    set_th_falling.check_returncode()
    
    #should be rougly at 170HZ such that we can measure between 5 and 1000 rpm with 400 samples (set to 98 which should come out to this frequency)
    set_sr = subprocess.run([f"{DIRECTORY}/micromini-tool", "set-ain-rate", set_sampling_rate], capture_output=False)
    set_sr.check_returncode()

    sp_rising = subprocess.run([f"{DIRECTORY}/micromini-tool", "get-ain-nrising"], capture_output=True)
    sp_rising.check_returncode()
    sp_falling = subprocess.run([f"{DIRECTORY}/micromini-tool", "get-ain-nfalling"], capture_output=True)
    sp_falling.check_returncode()
    sp_samples = subprocess.run([f"{DIRECTORY}/micromini-tool", "get-ain-num-samples"], capture_output=True)
    sp_samples.check_returncode()
    sp_sampling_rate = subprocess.run([f"{DIRECTORY}/micromini-tool", "get-ain-rate"], capture_output=True)
    sp_sampling_rate.check_returncode()

    rising_crossings = float(sp_rising.stdout.decode("utf-8").strip().split(":")[1].strip())
    falling_crossings = float(sp_falling.stdout.decode("utf-8").strip().split(":")[1].strip())
    num_samples = float(sp_samples.stdout.decode("utf-8").strip())

    #"Get AIN rate configuration (bits 0-2: ADC prescaler, bits 3-7 SAMPLELEN)"
    raw = int(sp_sampling_rate.stdout.decode("utf-8").strip().split(":")[1].strip())

    prescaler_bits = raw & 0b00000111
    samplelen_bits = (raw >> 3) & 0b00011111

    prescaler_dividers = [2, 4, 8, 16, 32, 64, 128, 256]
    divider = prescaler_dividers[prescaler_bits]

    adc_clock_hz = 32768
    total_cycles = samplelen_bits + 13
    sampling_rate_hz = adc_clock_hz / (divider * total_cycles)

    seconds_per_minute = 60
    crossings_per_rotation = 5
    rpm_rising = rising_crossings * sampling_rate_hz * seconds_per_minute / (crossings_per_rotation * num_samples)
    #rpm_falling = falling_crossings * sampling_rate * seconds_per_minute / (crossings_per_rotation * num_samples)

    rpm_dict = {}
    rpm_dict['n_rising'] = rising_crossings
    rpm_dict['n_falling'] = falling_crossings
    rpm_dict['n_samples'] = num_samples
    rpm_dict['sampling_rate_hz'] = sampling_rate_hz
    rpm_dict['rpm_rising'] = rpm_rising

    return rpm_dict

def read_ain():
    while True:
        sp = subprocess.run([f"{DIRECTORY}/micromini-tool", "get-ain-ready"], capture_output=True)

        m = re.match(b"get-ain-ready: ([0,1])\n", sp.stdout)
        if int(m.group(1)):
            sp = subprocess.run([f"{DIRECTORY}/micromini-tool", "get-ain-ready"], capture_output=True)
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


def parse_data(data, rpm_dict):
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
        "wind_A": float(wind_match.group(2))
        }

    combined_data = {**data, **rpm_dict}

    return combined_data


def log_power():
    time_interval_between_measurments = 20  # sec
    host = get_hostname()

    counter = 0
    while True:
        fname = get_file_name(prefix=f"/data/power/{host}_power_")

        try:
            measure()
            time.sleep(1)

            data = read_measurement()
            rpm_dict = read_rpm(set_threshold_value = '250', set_sampling_rate = '98')

            data = parse_data(data, rpm_dict)
            write_to_file(fname, data)
        except subprocess.CalledProcessError as e:
            print(e)

        time.sleep(time_interval_between_measurments)

        if counter % (3600 // time_interval_between_measurments) == 0:
            copy_data()
            reset()

        counter += 1


def copy_data():

    cmd = ["rsync -rav /data/power/*_power_*txt 10.1.0.1:/data/power-logs/"]

    sp = subprocess.run(cmd, capture_output=True, shell=True)
    print("Output:", sp.stdout, "\n", "Error:", sp.stderr)
    sp.check_returncode()



if __name__ == "__main__":

    # mode = sys.argv[1]

    # if mode not in ["log", "copy"]:
    #     sys.exit("The first argument must be either 'power' or'copy'. Exit ... ")

    # if mode == "log":
    #     log_power()

    # if mode == "copy":
    #     copy_data()

    log_power()