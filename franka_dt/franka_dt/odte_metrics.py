"""Utility functions for calculating Digital Twin entanglement metrics."""

import math


def calculate_timeliness(
    generation_timestamp,
    reception_timestamp,
    processing_time_seconds,
    desired_timeliness_seconds,
):
    """Return 1 if an update meets the timeliness threshold, otherwise 0."""
    values = (
        generation_timestamp,
        reception_timestamp,
        processing_time_seconds,
        desired_timeliness_seconds,
    )

    if not all(math.isfinite(value) for value in values):
        raise ValueError('I valori temporali devono essere finiti.')

    if generation_timestamp <= 0:
        raise ValueError('Il timestamp di generazione deve essere positivo.')

    if reception_timestamp < generation_timestamp:
        raise ValueError('La ricezione precede la generazione.')

    if processing_time_seconds < 0 or desired_timeliness_seconds <= 0:
        raise ValueError('I tempi devono rispettare i limiti consentiti.')

    elapsed = (
        reception_timestamp
        - generation_timestamp
        + processing_time_seconds
    )

    return int(elapsed <= desired_timeliness_seconds)


def calculate_reliability(measured_rate_hz, expected_rate_hz):
    """Return normalized update-rate reliability in the range [0, 1]."""
    if (
        not math.isfinite(measured_rate_hz)
        or not math.isfinite(expected_rate_hz)
    ):
        raise ValueError('Le frequenze devono essere finite.')

    if (
        measured_rate_hz < 0
        or expected_rate_hz <= 0
    ):
        raise ValueError('La frequenza misurata non può essere negativa '
                         'e quella attesa deve essere positiva.')

    return min(1.0, measured_rate_hz / expected_rate_hz)


def calculate_odte(timeliness, reliability, availability):
    """Return the ODTE product T * R * A after validating its components."""
    values = (timeliness, reliability, availability)

    if not all(math.isfinite(value) for value in values):
        raise ValueError('Le metriche ODTE devono essere finite.')

    if not all(0.0 <= value <= 1.0 for value in values):
        raise ValueError('Le metriche ODTE devono essere comprese tra 0 e 1.')

    return timeliness * reliability * availability
