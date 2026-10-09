"""Runtime form of the existing Jaesung per-SKU capacity assumptions.

Values and formula are unchanged from tools/prototypes/site_params/capacity.py.
This is an opt-in model, not a carton compression measurement or safety claim.
"""

from dataclasses import replace
import math


GRADES = {
    "single_B": (4250.0, 0.0030),
    "single_C": (5250.0, 0.0040),
    "double_BC": (8500.0, 0.0070),
}
WEIGHT_CLASS = ((5.0, "single_B"), (15.0, "single_C"), (math.inf, "double_BC"))
HUMIDITY = {50: 1.00, 70: 0.80, 80: 0.68, 90: 0.48}
STORAGE_DAYS = {10: 0.63, 30: 0.59, 90: 0.55, 180: 0.50}
HANDLING = 0.80
RESIDUAL_SF = 1.5
SITES = {
    "general": {"rh": 50, "days": 10, "label": "일반 출고 창고 (습도 50%, 10일 이내)"},
    "humid": {"rh": 80, "days": 10, "label": "다습 창고 (습도 80%)"},
    "long": {"rh": 70, "days": 90, "label": "장기 보관 (습도 70%, 90일)"},
}


def bct_n(dx, dy, ect, thickness):
    return 5.87 * ect * math.sqrt(thickness * 2.0 * (dx + dy))


def grade_for(weight_max_kg):
    for limit, grade in WEIGHT_CLASS:
        if weight_max_kg <= limit:
            return grade
    return WEIGHT_CLASS[-1][1]


def site_factor(site):
    s = SITES[site]
    return HUMIDITY[s["rh"]] * STORAGE_DAYS[s["days"]] * HANDLING / RESIDUAL_SF


def allowable_capacity_n(size, weight_max_kg, site="general"):
    """Same rounding and weight-class boundaries as the preserved prototype."""
    ect, thickness = GRADES[grade_for(weight_max_kg)]
    return round(bct_n(size.x, size.y, ect, thickness) * site_factor(site), 3)


def apply_site_capacity(catalog, weight_ranges, site="general"):
    """Return a catalog with only the allowable top loads replaced."""
    if site not in SITES:
        raise ValueError(f"unknown site capacity assumption: {site}")
    return {sku: replace(spec, top_load_capacity_n=allowable_capacity_n(
                spec.size, weight_ranges[sku][1], site))
            for sku, spec in catalog.items()}
