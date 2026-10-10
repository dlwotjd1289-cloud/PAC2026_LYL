from pipeline.run import decision_summary


def test_active_eight_second_budget_does_not_count_historical_seven_second_overrun():
    rows = [{"elapsed_s": t} for t in (7.5, 8.0, 8.01)]
    result = decision_summary(rows, 8.0)
    assert result["budget_s"] == 8.0
    assert result["over_budget"] == 1
    assert result["over_7s"] == 3


def test_disabled_budget_keeps_timing_without_inventing_a_deadline():
    assert decision_summary([{"elapsed_s": 12.0}], None)["over_budget"] == 0
    assert decision_summary([], 8.0)["max_s"] is None
