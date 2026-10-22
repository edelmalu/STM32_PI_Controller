import csv
import json
import math
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.pyplot as plt
import serial


PORT = "COM6"
BAUDRATE = 115200

DT = 0.01
SIMULATION_TIME = 12.0
SIMULATION_STEPS = round(SIMULATION_TIME / DT)
SYSTEM_CORE_CLOCK_HZ = 96_000_000
DEFAULT_KP = 0.08
DEFAULT_KI = 5.0

MAX_RPM = 1500.0
PROCESS_TIME_CONSTANT = 0.5
SETPOINT_RPM = 1200.0

DISTURBANCE_TIME = 5.0
DISTURBANCE_MODE = "pulse"  # "off", "pulse" oder "persistent"
DISTURBANCE_DURATION = 0.1  # Wird nur im pulse-Modus verwendet
DISTURBANCE_LEVELS = (200.0, 100.0, 50.0, 25.0, 10.0)

RESULTS_DIRECTORY = Path(__file__).resolve().parent / "results/integral/10.0"


def save_run_metadata(path, metadata):
    """Schreibt Konfiguration und Status lesbar neben die CSV und den Plot."""
    with path.open("w", encoding="utf-8") as metadata_file:
        json.dump(metadata, metadata_file, indent=2, ensure_ascii=False)
        metadata_file.write("\n")


def set_speed_and_wait_for_ack(stm32, setpoint, timeout_s=5.0):
    return send_command_and_wait_for_ack(
        stm32,
        f"SET_SPEED,{setpoint:.1f}",
        "OK,SET_SPEED,",
        timeout_s,
    )


def send_command_and_wait_for_ack(stm32, command, expected_prefix, timeout_s=5.0):
    stm32.write(f"{command}\n".encode("ascii"))
    deadline = time.monotonic() + timeout_s
    discarded_lines = 0
    last_response = ""

    while time.monotonic() < deadline:
        response = stm32.readline().decode(
            "ascii", errors="replace"
        ).strip()
        if not response:
            continue
        if response.startswith(expected_prefix):
            return response
        if response.startswith("ERR,"):
            raise RuntimeError(f"STM32 hat den Befehl abgelehnt: {response}")

        # Nach dem Reset kann zuerst eine mitten im UART-Telegramm beginnende
        # Zeile eintreffen; außerdem sendet die Firmware regelmäßig Timerdaten.
        discarded_lines += 1
        last_response = response

    raise TimeoutError(
        f"Keine Bestätigung für {command!r} vom STM32 innerhalb von "
        f"{timeout_s:.1f} s. Verworfen: {discarded_lines} Zeilen; "
        f"letzte Zeile: {last_response!r}"
    )


def prompt_gain(name, default):
    raw_value = input(f"{name} [{default:g}, 0 bis 1000]: ").strip()
    value = default if not raw_value else float(raw_value)
    if not math.isfinite(value) or not 0.0 <= value <= 1000.0:
        raise ValueError(f"{name} muss zwischen 0 und 1000 liegen")
    return value


def create_plot(run_directory, times, setpoints, measurements,
                load_disturbances, control_outputs, disturbance_mode,
                disturbance_duration):
    """Erzeugt den Plot und speichert ihn im Ergebnisordner dieses Laufs."""
    figure, (ax_values, ax_zoom, ax_disturbance) = plt.subplots(
        3,
        1,
        figsize=(10, 9),
        gridspec_kw={"height_ratios": [3, 2, 1]},
    )
    ax_values.plot(times, setpoints, label="Sollwert", linestyle="--")
    ax_values.plot(times, measurements, label="Istwert", color="tab:blue")
    ax_values.axvline(DISTURBANCE_TIME, color="red", linestyle=":")
    ax_values.set_ylabel("Wert")
    ax_values.set_xlabel("Simulationszeit [s]")
    ax_values.set_title("PI-Regler – Sollwert, Istwert und Laststörung")
    ax_values.grid(True)
    ax_values.legend()

    zoom_start = max(0.0, DISTURBANCE_TIME - 0.5)
    zoom_end = min(SIMULATION_TIME, DISTURBANCE_TIME + 1.5)
    zoom_indices = [
        i for i, sample_time in enumerate(times)
        if zoom_start <= sample_time <= zoom_end
    ]
    if zoom_indices:
        zoom_times = [times[i] for i in zoom_indices]
        zoom_measurements = [measurements[i] for i in zoom_indices]
        zoom_setpoints = [setpoints[i] for i in zoom_indices]
        zoom_outputs = [control_outputs[i] for i in zoom_indices]
        ax_zoom.plot(
            zoom_times, zoom_setpoints, label="Sollwert", linestyle="--"
        )
        ax_zoom.plot(
            zoom_times, zoom_measurements, label="Istwert", color="tab:blue"
        )
        ax_zoom.set_xlim(zoom_start, zoom_end)
        zoom_min = min(zoom_measurements + zoom_setpoints)
        zoom_max = max(zoom_measurements + zoom_setpoints)
        zoom_margin = max(2.0, (zoom_max - zoom_min) * 0.2)
        ax_zoom.set_ylim(zoom_min - zoom_margin, zoom_max + zoom_margin)

        ax_output = ax_zoom.twinx()
        ax_output.plot(
            zoom_times,
            zoom_outputs,
            label="Reglerausgang u",
            color="tab:orange",
            alpha=0.8,
        )
        ax_output.set_ylabel("Stellgröße u")
        ax_output.set_ylim(0.0, 1.05)
        output_lines, output_labels = ax_output.get_legend_handles_labels()
        value_lines, value_labels = ax_zoom.get_legend_handles_labels()
        ax_zoom.legend(value_lines + output_lines, value_labels + output_labels)

    ax_zoom.axvline(DISTURBANCE_TIME, color="red", linestyle=":")
    ax_zoom.set_ylabel("Istwert")
    ax_zoom.set_title("Vergrößerung um den Störzeitpunkt")
    ax_zoom.grid(True)

    disturbance_label = {
        "off": "Keine Laststörung",
        "pulse": f"Laststörimpuls ({disturbance_duration:g} s)",
        "persistent": "Dauerhafte Laststörung",
    }[disturbance_mode]
    ax_disturbance.step(
        times,
        load_disturbances,
        where="post",
        label=disturbance_label,
        color="tab:green",
        linewidth=2,
    )
    if disturbance_mode != "off":
        ax_disturbance.axvline(DISTURBANCE_TIME, color="red", linestyle=":")
    ax_disturbance.set_xlabel("Simulationszeit [s]")
    ax_disturbance.set_ylabel("Störwert")
    ax_disturbance.grid(True)
    ax_disturbance.legend()
    figure.tight_layout()

    plot_path = run_directory / "plot.png"
    figure.savefig(plot_path, dpi=150)
    plt.close(figure)
    return plot_path


def print_run_summary(times, measurements, receive_times, timer_counts,
                      timer_period_cycles, control_outputs, load_disturbances,
                      disturbance_mode):
    summary = {
        "samples_received": len(measurements),
        "samples_requested": SIMULATION_STEPS,
        "disturbance": {
            "mode": disturbance_mode,
            "active": False,
            "first_active_time_s": None,
        },
        "regulation": {
            "settling_time_s": None,
            "tolerance_band": 0.02 * SETPOINT_RPM,
            "maximum_value": None,
            "overshoot": None,
            "overshoot_percent": None,
            "steady_mean_last_2s": None,
            "steady_error_last_2s": None,
            "max_abs_deviation_first_1s_after_disturbance": None,
        },
        "disturbance_response_first_1_5s": None,
        "uart_telemetry": None,
        "timer_dwt": None,
    }
    print(f"\nEmpfangene Simulationsschritte: {len(measurements)} / {SIMULATION_STEPS}")

    active_disturbance_times = [
        sample_time
        for sample_time, disturbance in zip(times, load_disturbances)
        if disturbance != 0.0
    ]
    if active_disturbance_times:
        summary["disturbance"]["active"] = True
        summary["disturbance"]["first_active_time_s"] = active_disturbance_times[0]
        print(
            f"Laststörung erstmals aktiv bei "
            f"{active_disturbance_times[0]:.3f} s "
            f"(Modus: {disturbance_mode}, Start: {DISTURBANCE_TIME:.3f} s)"
        )
        response_indices = [
            i for i, sample_time in enumerate(times)
            if DISTURBANCE_TIME <= sample_time <= DISTURBANCE_TIME + 1.5
        ]
        if response_indices:
            response_values = [measurements[i] for i in response_indices]
            response_outputs = [control_outputs[i] for i in response_indices]
            summary["disturbance_response_first_1_5s"] = {
                "measurement_min": min(response_values),
                "measurement_max": max(response_values),
                "controller_output_u_min": min(response_outputs),
                "controller_output_u_max": max(response_outputs),
            }
            print(
                f"Istwert nach Störung: min {min(response_values):.2f}, "
                f"max {max(response_values):.2f}; "
                f"Reglerausgang u: {min(response_outputs):.4f} bis "
                f"{max(response_outputs):.4f}"
            )
    else:
        print(
            f"Laststörung in den erfassten Daten nicht aktiv geworden "
            f"(konfiguriert ab {DISTURBANCE_TIME:.3f} s)"
        )

    print("\n--- Regelgüte ---")
    if not measurements:
        print("Keine Messwerte vorhanden.")
    else:
        tolerance = 0.02 * SETPOINT_RPM
        settling_time = None

        for i in range(len(measurements)):
            if all(
                abs(value - SETPOINT_RPM) <= tolerance
                for value in measurements[i:]
            ):
                settling_time = times[i]
                break

        max_value = max(measurements)
        overshoot = max(0.0, max_value - SETPOINT_RPM)
        overshoot_percent = (
            overshoot / SETPOINT_RPM * 100 if SETPOINT_RPM != 0 else 0.0
        )

        end_time = times[-1]
        steady_values = [
            value
            for sample_time, value in zip(times, measurements)
            if sample_time >= end_time - 2.0
        ]
        steady_mean = sum(steady_values) / len(steady_values)
        steady_error = SETPOINT_RPM - steady_mean
        summary["regulation"].update(
            {
                "settling_time_s": settling_time,
                "maximum_value": max_value,
                "overshoot": overshoot,
                "overshoot_percent": overshoot_percent,
                "steady_mean_last_2s": steady_mean,
                "steady_error_last_2s": steady_error,
            }
        )

        if settling_time is not None:
            print(f"Einschwingzeit (+/- 2 %): {settling_time:.2f} s")
        else:
            print("Einschwingzeit: Toleranzband nicht dauerhaft erreicht")

        print(f"Maximaler Wert: {max_value:.1f}")
        print(f"Überschwingen: {overshoot:.1f} ({overshoot_percent:.2f} %)")
        print(f"Mittlerer Wert am Ende: {steady_mean:.1f}")
        print(f"Stationäre Regelabweichung: {steady_error:.1f}")

        if active_disturbance_times:
            disturbance_window = [
                value
                for sample_time, value in zip(times, measurements)
                if DISTURBANCE_TIME <= sample_time <= DISTURBANCE_TIME + 1.0
            ]
            if disturbance_window:
                max_abs_deviation = max(
                    abs(value - SETPOINT_RPM) for value in disturbance_window
                )
                summary["regulation"][
                    "max_abs_deviation_first_1s_after_disturbance"
                ] = max_abs_deviation
                print(
                    f"Größte Abweichung vom Sollwert in der ersten Sekunde "
                    f"nach Störung: {max_abs_deviation:.2f}"
                )

    if len(receive_times) > 1:
        receive_intervals_ms = [
            (later - earlier) * 1000
            for earlier, later in zip(receive_times, receive_times[1:])
        ]
        tick_deltas = [
            (later - earlier) & 0xFFFFFFFF
            for earlier, later in zip(timer_counts, timer_counts[1:])
        ]
        missed_ticks = sum(max(delta - 1, 0) for delta in tick_deltas)
        duplicate_counts = sum(delta == 0 for delta in tick_deltas)
        timer_tick_span = (timer_counts[-1] - timer_counts[0]) & 0xFFFFFFFF
        observed_span = receive_times[-1] - receive_times[0]
        summary["uart_telemetry"] = {
            "receive_interval_mean_ms": statistics.mean(receive_intervals_ms),
            "receive_interval_median_ms": statistics.median(receive_intervals_ms),
            "receive_interval_min_ms": min(receive_intervals_ms),
            "receive_interval_max_ms": max(receive_intervals_ms),
            "missed_timer_telegrams": missed_ticks,
            "duplicate_timer_counts": duplicate_counts,
            "timer_tick_span": timer_tick_span,
            "pc_observed_ms_per_timer_tick": (
                observed_span * 1000 / timer_tick_span
                if timer_tick_span > 0
                else None
            ),
        }

        print("\n--- UART-Telemetrie am PC (nicht die reine TIM2-Zeit) ---")
        print(
            f"Empfangsabstände: Mittel "
            f"{statistics.mean(receive_intervals_ms):.2f} ms, "
            f"Median {statistics.median(receive_intervals_ms):.2f} ms, "
            f"Min {min(receive_intervals_ms):.2f} ms, "
            f"Max {max(receive_intervals_ms):.2f} ms"
        )
        print(f"Ausgelassene TIM2-Telegramme laut Zähler: {missed_ticks}")
        print(f"Doppelte TIM2-Zählerstände: {duplicate_counts}")

        measured_periods = [
            cycles for cycles in timer_period_cycles if cycles > 0
        ]
        if measured_periods:
            expected_period_cycles = SYSTEM_CORE_CLOCK_HZ * DT
            mean_period_cycles = statistics.mean(measured_periods)
            mean_period_ms = mean_period_cycles * 1000 / SYSTEM_CORE_CLOCK_HZ
            period_error_percent = (
                (mean_period_cycles - expected_period_cycles)
                / expected_period_cycles
                * 100
            )
            summary["timer_dwt"] = {
                "interval_count": len(measured_periods),
                "period_cycles_mean": mean_period_cycles,
                "period_cycles_min": min(measured_periods),
                "period_cycles_max": max(measured_periods),
                "period_mean_ms": mean_period_ms,
                "expected_period_cycles": expected_period_cycles,
                "period_error_percent": period_error_percent,
            }
            print("\n--- TIM2-Intervall im STM32 (DWT CPU-Zyklen) ---")
            print(f"Erfasste Intervalle: {len(measured_periods)}")
            print(
                f"Zyklen: Mittel {mean_period_cycles:.1f}, "
                f"Min {min(measured_periods)}, Max {max(measured_periods)}"
            )
            print(
                f"Entspricht bei {SYSTEM_CORE_CLOCK_HZ / 1_000_000:.0f} MHz: "
                f"{mean_period_ms:.4f} ms"
            )
            print(
                f"Soll: {expected_period_cycles:.0f} Zyklen "
                f"({DT * 1000:.2f} ms), "
                f"Abweichung: {period_error_percent:+.3f} %"
            )
        else:
            print(
                "\nKeine DWT-Zykluswerte empfangen. Für die STM32-interne "
                "Intervallmessung bitte die aktualisierte Firmware flashen."
            )

        if timer_tick_span > 0:
            measured_tick_ms = observed_span * 1000 / timer_tick_span
            print(f"TIM2-Zählerticks pro Empfangszeitraum: {timer_tick_span}")
            print(
                f"Empfangsabstand pro TIM2-Tick laut PC-Uhr: "
                f"{measured_tick_ms:.3f} ms"
            )
    return summary


def run_simulation(run_directory, kp_value, ki_value, disturbance_mode,
                   disturbance_rpm):
    run_directory.mkdir(parents=True, exist_ok=False)
    run_started_at = datetime.now(timezone.utc)

    csv_path = run_directory / "measurements.csv"
    run_data_path = run_directory / "run.json"
    plot_path = run_directory / "plot.png"
    run_data = {
        "status": "running",
        "started_at_utc": run_started_at.isoformat(),
        "simulation_duration_s": None,
        "wall_clock_duration_s": None,
        "samples_written": 0,
        "configuration": {
            "port": PORT,
            "baudrate": BAUDRATE,
            "dt_s_fallback": DT,
            "simulation_time_s": SIMULATION_TIME,
            "requested_steps": SIMULATION_STEPS,
            "system_core_clock_hz_for_dwt_conversion": SYSTEM_CORE_CLOCK_HZ,
            "controller_gains": {
                "kp": kp_value,
                "ki": ki_value,
            },
            "max_process_value": MAX_RPM,
            "process_time_constant_s": PROCESS_TIME_CONSTANT,
            "setpoint": SETPOINT_RPM,
            "disturbance": {
                "mode": disturbance_mode,
                "start_s": DISTURBANCE_TIME,
                "magnitude": disturbance_rpm,
                "duration_s": (
                    DISTURBANCE_DURATION
                    if disturbance_mode == "pulse"
                    else None
                ),
            },
        },
        "results": None,
        "files": {
            "measurements": csv_path.name,
            "run_data": run_data_path.name,
            "plot": plot_path.name,
        },
    }
    save_run_metadata(run_data_path, run_data)

    measured_value = 0.0
    times = []
    setpoints = []
    measurements = []
    load_disturbances = []
    control_outputs = []
    receive_times = []
    timer_counts = []
    timer_period_cycles = []
    simulation_elapsed = 0.0
    run_started_monotonic = time.monotonic()

    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as stm32:
            time.sleep(2)
            stm32.reset_input_buffer()

            def send_command(command):
                stm32.write((command + "\n").encode("ascii"))
                return stm32.readline().decode(
                    "ascii", errors="replace"
                ).strip()

            # Vor der Bestätigung können noch alte oder angeschnittene Timerzeilen ankommen.
            response = send_command_and_wait_for_ack(
                stm32,
                f"SET_PI,{kp_value:.6f},{ki_value:.6f}",
                "OK,SET_PI,",
            )
            print("STM32:", response)

            response = set_speed_and_wait_for_ack(stm32, SETPOINT_RPM)
            print("STM32:", response)

            start = time.monotonic()
            stm32.write(f"MEAS,{measured_value:.2f}\n".encode("ascii"))

            # Jede CSV-Zeile sofort schreiben, damit Daten bei Abbruch erhalten bleiben.
            with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(
                    [
                        "simulation_time_s",
                        "host_elapsed_s",
                        "tim2_count",
                        "timer_period_cycles",
                        "setpoint",
                        "measured_value",
                        "controller_output_u",
                        "disturbance",
                        "effective_target",
                    ]
                )
                csv_file.flush()

                # TIM2 sendet nach jeder 10-ms-Reglerberechnung einen Reglerwert.
                while len(measurements) < SIMULATION_STEPS:
                    response = stm32.readline().decode(
                        "ascii", errors="replace"
                    ).strip()
                    received_at = time.monotonic()

                    if not response.startswith("U,"):
                        raise RuntimeError(
                            f"Unerwartete oder fehlende STM32-Antwort: "
                            f"{response!r}"
                        )

                    parts = response.split(",")
                    if (
                        len(parts) >= 4
                        and parts[0] == "U"
                        and parts[2] == "TIM2"
                    ):
                        controller_output = float(parts[1])
                        tim2_count = int(parts[3])
                        period_cycles = (
                            int(parts[4]) if len(parts) >= 5 else 0
                        )
                    else:
                        raise ValueError(
                            f"Unerwartete STM32-Antwort: {response!r}"
                        )

                    step_dt = (
                        period_cycles / SYSTEM_CORE_CLOCK_HZ
                        if period_cycles > 0
                        else DT
                    )
                    simulation_elapsed += step_dt

                    disturbance_is_active = (
                        disturbance_mode != "off"
                        and simulation_elapsed >= DISTURBANCE_TIME
                    )
                    if disturbance_mode == "pulse":
                        disturbance_is_active = (
                            disturbance_is_active
                            and simulation_elapsed
                            < DISTURBANCE_TIME + DISTURBANCE_DURATION
                        )
                    load_disturbance = (
                        disturbance_rpm if disturbance_is_active else 0.0
                    )

                    # u ist normiert; MAX_RPM skaliert ihn auf den virtuellen Prozesswert.
                    target_value = MAX_RPM * controller_output
                    effective_target = max(
                        0.0,
                        min(MAX_RPM, target_value - load_disturbance),
                    )
                    measured_value += (
                        (effective_target - measured_value)
                        * step_dt
                        / PROCESS_TIME_CONSTANT
                    )
                    measured_value = max(
                        0.0, min(MAX_RPM, measured_value)
                    )

                    stm32.write(
                        f"MEAS,{measured_value:.2f}\n".encode("ascii")
                    )

                    receive_times.append(received_at)
                    timer_counts.append(tim2_count)
                    timer_period_cycles.append(period_cycles)
                    times.append(simulation_elapsed)
                    setpoints.append(SETPOINT_RPM)
                    measurements.append(measured_value)
                    load_disturbances.append(load_disturbance)
                    control_outputs.append(controller_output)

                    csv_writer.writerow(
                        [
                            f"{simulation_elapsed:.9f}",
                            f"{received_at - start:.9f}",
                            tim2_count,
                            period_cycles,
                            f"{SETPOINT_RPM:.6f}",
                            f"{measured_value:.6f}",
                            f"{controller_output:.6f}",
                            f"{load_disturbance:.6f}",
                            f"{effective_target:.6f}",
                        ]
                    )
                    csv_file.flush()

        actual_duration = time.monotonic() - start
        run_data["simulation_duration_s"] = actual_duration
        print(f"\nTatsächliche Simulationsdauer: {actual_duration:.3f} s")

        create_plot(
            run_directory,
            times,
            setpoints,
            measurements,
            load_disturbances,
            control_outputs,
            disturbance_mode,
            DISTURBANCE_DURATION,
        )
        summary = print_run_summary(
            times,
            measurements,
            receive_times,
            timer_counts,
            timer_period_cycles,
            control_outputs,
            load_disturbances,
            disturbance_mode,
        )
        run_data["results"] = summary
        run_data["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        save_run_metadata(run_data_path, run_data)
        print(f"\nLaufdaten und Zusammenfassung gespeichert: {run_data_path}")
        print(f"Alle Laufdateien gespeichert unter: {run_directory}")

        run_data["status"] = "completed"
    except Exception as error:
        run_data["status"] = "failed"
        run_data["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        if run_data["status"] == "running":
            run_data["status"] = "interrupted"
        run_data["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        run_data["wall_clock_duration_s"] = (
            time.monotonic() - run_started_monotonic
        )
        run_data["samples_written"] = len(measurements)
        save_run_metadata(run_data_path, run_data)


def main():
    if DISTURBANCE_MODE not in ("off", "pulse", "persistent"):
        raise ValueError(
            "DISTURBANCE_MODE muss 'off', 'pulse' oder 'persistent' sein"
        )

    print("PI-Reglerparameter eingeben (Enter übernimmt den Standardwert).")
    kp_value = prompt_gain("kp", DEFAULT_KP)
    ki_value = prompt_gain("ki", DEFAULT_KI)

    batch_started_at = datetime.now().astimezone()
    batch_id = batch_started_at.strftime("%Y%m%d_%H%M%S_%f")
    batch_directory = RESULTS_DIRECTORY / f"batch_{batch_id}"
    batch_directory.mkdir(parents=True, exist_ok=False)

    print(
        f"\nStarte Versuchsreihe: Modus={DISTURBANCE_MODE}, "
        f"kp={kp_value:g}, ki={ki_value:g}"
    )
    for index, disturbance_rpm in enumerate(DISTURBANCE_LEVELS, start=1):
        run_name = f"{DISTURBANCE_MODE}_{disturbance_rpm:g}"
        run_directory = batch_directory / run_name
        print(
            f"\n=== Lauf {run_name} ({index}/{len(DISTURBANCE_LEVELS)}) ==="
        )
        run_simulation(
            run_directory,
            kp_value,
            ki_value,
            DISTURBANCE_MODE,
            disturbance_rpm,
        )

    print(f"\nVersuchsreihe abgeschlossen: {batch_directory}")


# Beim Import werden nur Konfiguration und Funktionen geladen, nicht der COM-Port geöffnet.
if __name__ == "__main__":
    main()
