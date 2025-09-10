#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import csv
from datetime import datetime
import sys

def read_csv_data(filepath): #funzione per leggere i file CSV
    
    raw_timestamps = [] #lista per salvare i timestamp originali
    xs = []
    ys = []
    zs = []
    try:
        with open(filepath, 'r') as file: #apre il csv in modalità di lettura
            reader = csv.reader(file)
            header = next(reader) #salta l'intestazione

            for row in reader:
                #converte i timestamp in datetime e i valori XYZ in float
                raw_timestamps.append(datetime.strptime(row[0], "%Y-%m-%d %H:%M:%S.%f"))
                xs.append(float(row[1]))
                ys.append(float(row[2]))
                zs.append(float(row[3]))

    except FileNotFoundError:
        print(f"Errore: Il file '{filepath}' non è stato trovato.")
        return np.array([]), np.array([]), np.array([]), np.array([])
    except Exception as e:
        print(f"Errore durante la lettura del file '{filepath}': {e}")
        return np.array([]), np.array([]), np.array([]), np.array([])

    if not raw_timestamps:
        print(f"Avviso: Il file '{filepath}' è vuoto o non contiene dati validi.")
        return np.array([]), np.array([]), np.array([]), np.array([])

    #calcola il tempo in secondi relativo al primo timestamp
    start_time = raw_timestamps[0]
    times_sec = np.array([(t - start_time).total_seconds() for t in raw_timestamps])

    return times_sec, np.array(xs), np.array(ys), np.array(zs)

def plot_robot_errors_no_pandas(target_pos_file, next_pos_file, reached_pos_file): #legge i CSV e genera i grafici di errore
    
    #legge i dati da tutti i CSV
    print(f"Caricamento dati da '{target_pos_file}'...")
    time_target, target_xs, target_ys, target_zs = read_csv_data(target_pos_file)
    print(f"Caricamento dati da '{next_pos_file}'...")
    time_next, next_xs, next_ys, next_zs = read_csv_data(next_pos_file)
    print(f"Caricamento dati da '{reached_pos_file}'...")
    time_reached, reached_xs, reached_ys, reached_zs = read_csv_data(reached_pos_file)

    #verifica validità dei dati
    if len(time_target) == 0 or len(time_next) == 0 or len(time_reached) == 0:
        print("Uno o più file CSV sono vuoti o non leggibili. Impossibile generare i grafici.")
        return

    #sincronizzazione dei dati, usando la lunghezza minima tra i tre file 
    min_len = min(len(time_target), len(time_next), len(time_reached))

    if min_len == 0:
        print("Nessun dato comune tra i file CSV dopo il caricamento. Impossibile generare i grafici.")
        return

    #allinea tutti gli array alla stessa lunghezza (min_len)
    time_target = time_target[:min_len]
    target_xs, target_ys, target_zs = target_xs[:min_len], target_ys[:min_len], target_zs[:min_len]

    time_next = time_next[:min_len]
    next_xs, next_ys, next_zs = next_xs[:min_len], next_ys[:min_len], next_zs[:min_len]

    time_reached = time_reached[:min_len]
    reached_xs, reached_ys, reached_zs = reached_xs[:min_len], reached_ys[:min_len], reached_zs[:min_len]

    #calcolo degli errori di interesse
    error_x_target_reached = next_xs - reached_xs
    error_y_target_reached = next_ys - reached_ys
    error_z_target_reached = next_zs - reached_zs

    total_error_target_reached = np.sqrt(
        error_x_target_reached**2 +
        error_y_target_reached**2 +
        error_z_target_reached**2
    )

    error_x_next_target = next_xs - target_xs
    error_y_next_target = next_ys - target_ys
    error_z_next_target = next_zs - target_zs

    #generazione dei grafici di interesse
    plt.figure(figsize=(12, 7))
    plt.plot(time_target, error_x_target_reached, label='Errore X', color='red', linewidth=1.5)
    plt.plot(time_target, error_y_target_reached, label='Errore Y', color='green', linewidth=1.5)
    plt.plot(time_target, error_z_target_reached, label='Errore Z', color='blue', linewidth=1.5)
    plt.xlabel('Tempo (s)', fontsize=12)
    plt.ylabel('Errore (m)', fontsize=12)
    plt.title('Errore Individuale: (Posizione calcolata - Posizione Raggiunta) per Asse', fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()

    
    plt.figure(figsize=(12, 7))
    plt.plot(time_target, total_error_target_reached, label='Errore Totale', color='red', linewidth=2)
    plt.xlabel('Tempo (s)', fontsize=12)
    plt.ylabel('Distanza Euclidea (m)', fontsize=12)
    plt.title('Errore Totale: Distanza Euclidea (Posizione calcolata - Posizione Raggiunta)', fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()

  
    plt.figure(figsize=(12, 7))
    plt.plot(time_target, error_x_next_target, label='Errore X', color='red', linewidth=1.5)
    plt.plot(time_target, error_y_next_target, label='Errore Y', color='green', linewidth=1.5)
    plt.plot(time_target, error_z_next_target, label='Errore Z', color='blue', linewidth=1.5)
    plt.xlabel('Tempo (s)', fontsize=12)
    plt.ylabel('Errore (m)', fontsize=12)
    plt.title('Errore di Spostamento: (Next Pos - Target Pos) per Asse', fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()

def main(args=None):
    
    target_csv = "target_pos_log.csv"
    next_csv = "next_pos_log.csv"
    reached_csv = "end_effector_reached_pos_log.csv"

    plot_robot_errors_no_pandas(target_csv, next_csv, reached_csv)

if __name__ == '__main__':
    main()