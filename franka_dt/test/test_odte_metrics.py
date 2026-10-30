"""Unit tests for ODTE metric calculation functions."""

from franka_dt.odte_metrics import (
    calculate_odte,
    calculate_reliability,
    calculate_timeliness,
)

import pytest


def test_timeliness_within_threshold():
    """Accept an update received within the desired time threshold."""
    result = calculate_timeliness(10.0, 10.03, 0.01, 0.05)

    assert result == 1


def test_timeliness_exceeds_threshold():
    """Reject an update exceeding the desired time threshold."""
    result = calculate_timeliness(10.0, 10.06, 0.01, 0.05)

    assert result == 0


def test_timeliness_exactly_at_threshold():
    """Accept an update exactly at the time threshold."""
    result = calculate_timeliness(10.0, 10.04, 0.01, 0.05)

    assert result == 1


@pytest.mark.parametrize(
    'generation, reception, processing, desired',
    [
        (0.0, 10.0, 0.01, 0.05),
        (10.0, 9.0, 0.01, 0.05),
        (10.0, 10.1, -0.01, 0.05),
        (10.0, 10.1, 0.01, 0.0),
        (float('nan'), 10.1, 0.01, 0.05),
        (10.0, float('inf'), 0.01, 0.05),
    ],
)
def test_timeliness_rejects_invalid_values(
    generation, reception, processing, desired
):
    """Reject invalid timestamps and timing values."""
    with pytest.raises(ValueError):
        calculate_timeliness(
            generation, reception, processing, desired
        )


def test_reliability_below_expected_rate():
    """Normalize an update rate below the expected rate."""
    assert calculate_reliability(50.0, 100.0) == 0.5


def test_reliability_at_expected_rate():
    """Return full reliability at the expected rate."""
    assert calculate_reliability(100.0, 100.0) == 1.0


def test_reliability_above_expected_rate_is_capped():
    """Cap reliability at one when the rate exceeds expectations."""
    assert calculate_reliability(120.0, 100.0) == 1.0


def test_reliability_with_zero_measured_rate():
    """Return zero reliability when no updates are measured."""
    assert calculate_reliability(0.0, 100.0) == 0.0


@pytest.mark.parametrize(
    'measured, expected',
    [
        (-1.0, 100.0),
        (50.0, 0.0),
        (50.0, -100.0),
        (float('nan'), 100.0),
        (50.0, float('inf')),
    ],
)
def test_reliability_rejects_invalid_values(measured, expected):
    """Reject invalid measured and expected update rates."""
    with pytest.raises(ValueError):
        calculate_reliability(measured, expected)


def test_odte_product():
    """Calculate ODTE as the product of its three components."""
    assert calculate_odte(0.9, 0.8, 1.0) == pytest.approx(0.72)


def test_odte_is_zero_if_one_component_is_zero():
    """Return zero when any ODTE component is zero."""
    assert calculate_odte(1.0, 0.0, 1.0) == 0.0


def test_odte_is_one_when_all_components_are_one():
    """Return one when all ODTE components equal one."""
    assert calculate_odte(1.0, 1.0, 1.0) == 1.0


@pytest.mark.parametrize(
    'timeliness, reliability, availability',
    [
        (-0.1, 0.8, 1.0),
        (1.1, 0.8, 1.0),
        (0.9, float('nan'), 1.0),
        (0.9, 0.8, float('inf')),
    ],
)
def test_odte_rejects_invalid_values(
    timeliness, reliability, availability
):
    """Reject ODTE components outside the valid range."""
    with pytest.raises(ValueError):
        calculate_odte(timeliness, reliability, availability)
