import optuna
import argparse
from fitness_calculation import fitness

parser = argparse.ArgumentParser()
parser.add_argument("--csv", help="Path to CSV file (eg: /../LogReader/logfile.csv)", required=True)
args = parser.parse_args()

def write_params(params, trial_number):
    with open(f"params_trial_{trial_number}.txt", 'w') as f:
        f.write("// Optuna Trial " + str(trial_number) + "\n")
        f.write(f"int line_escape_duration = {params['line_escape_duration']};\n")
        f.write(f"int line_threshold = {params['line_threshold']};\n")
        f.write(f"int orbit_speed = {params['orbit_speed']};\n")
        f.write(f"int angle_precision = {params['angle_precision']};\n")
        f.write(f"int dist_threshold = {params['dist_threshold']};\n")

def objective(trial):
    params = {
        'line_escape_duration': trial.suggest_int('line_escape_duration', 150, 600),
        'line_threshold':       trial.suggest_int('line_threshold', -30, -5),
        'orbit_speed':          trial.suggest_int('orbit_speed', 15, 40),
        'angle_precision':      trial.suggest_int('angle_precision', 8, 25),
        'dist_threshold':       trial.suggest_int('dist_threshold', 150, 300),
    }
    
    write_params(params, trial.number)
    print(f"\n>>> Params geschrieben nach params_trial_{trial.number}.txt")
    input(">>> Manuell eintragen, flashen, laufen lassen, Log Dump → Enter")
    
    return fitness(args.csv)

study = optuna.create_study(direction='minimize')
study.optimize(objective, n_trials=20)

print("\nBeste Parameter:")
print(study.best_params)