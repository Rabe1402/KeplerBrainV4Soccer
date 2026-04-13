import pandas as pd
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--csv", help="Path to CSV file (eg: /../LogReader/logfile.csv)", required=True)
args = parser.parse_args()


def fitness(arg_csv):
    df = pd.read_csv(arg_csv)
    
    # State-Flapping
    transitions = df[df['component'] == 'STATE_TRANSITION']
    flaps = (transitions['timestamp'].diff() < 200).sum()
    
    # Line Escape Dauer
    escapes = df[df['component'] == 'LINE_ESCAPE_DONE']
    avg_escape = escapes['duration_ms'].mean()
    
    # Zeit in Search-State
    search_time = df[df['state'] == 0]['timestamp'].count()
    
    # ACC RMS
    acc_rms = (df['acc_x']**2 + df['acc_y']**2).mean()**0.5

    # BAD_EVENTs gewichten
    bad_events = df[df['component'] == 'BAD_EVENT']
    bad_event_score = len(bad_events) * 20  # jedes manuelle Flag = +20 Penalty

    score = (flaps * 10) + (avg_escape * 0.5) + (search_time * 0.1) + (acc_rms * 0.2) + bad_event_score
    return score

