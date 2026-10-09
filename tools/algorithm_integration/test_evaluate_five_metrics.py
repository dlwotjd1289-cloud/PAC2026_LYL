"""Synthetic arithmetic/independence tests; no planning or simulation."""
from copy import deepcopy
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parent))
from evaluate_five_metrics import evaluate_record, EFF_WEIGHTS


def record():
    return {"geometry_features":dict.fromkeys(EFF_WEIGHTS,0.),"packing_fraction":.4,
            "stability_components":dict.fromkeys(("support","interlock","tilt","side"),1.),
            "load":{"max_load_ratio":0.,"cog_xyz_m":[.5,.5,.4]},
            "timing":{"decision_s":.2,"robot_estimate_s":8.},
            "execution":{"robot_ok":True,"clearance_m":.01,"result_ok":True}}


def test_original_formula_exact_endpoints_and_no_invented_total():
    r=evaluate_record(record())
    assert r['stability']['S']['value']==1.
    assert r['stability']['phi_S']['value']==15.5
    assert r['load']['phi_L']['value']==4.25
    assert r['legacy_cost']['value']==-19.75
    assert r['legacy_cost']['is_five_category_total'] is False
    assert all(v is False for v in r['performed'].values())


def test_features_use_original_seven_weights_once():
    data=record();data['geometry_features']={key:i for i,key in enumerate(EFF_WEIGHTS)}
    actual=evaluate_record(data)['space']['legacy_efficiency_cost']['value']
    assert actual==sum(i*weight for i,weight in enumerate((2,2,2,1.5,4,1.5,.5)))


@pytest.mark.parametrize('ratio,margin,reward',[(0.,1.,4.25),(.5,.5,4.),(1.,0.,0.),(1.4,0.,0.)])
def test_load_clipping_and_nonlinear_boundary(ratio,margin,reward):
    data=record();data['load']['max_load_ratio']=ratio
    out=evaluate_record(data)['load']
    assert out['L']['value']==margin
    assert out['phi_L']['value']==reward


@pytest.mark.parametrize('component,reward',[(0.,0.),(.75,15.),(1.,15.5)])
def test_stability_nonlinear_boundary(component,reward):
    data=record();data['stability_components']=dict.fromkeys(data['stability_components'],component)
    assert evaluate_record(data)['stability']['phi_S']['value']==pytest.approx(reward)


def test_time_cog_execution_do_not_change_original_cost_or_mutate_input():
    data=record();before=deepcopy(data);first=evaluate_record(data)
    assert data==before
    data['timing']={'decision_s':99.,'robot_estimate_s':900.}
    data['load']['cog_xyz_m']=[-.5,.1,2.]
    data['execution']={'robot_ok':False,'clearance_m':-.1,'result_ok':False}
    second=evaluate_record(data)
    assert second['legacy_cost']==first['legacy_cost']
    assert second['execution_feasibility']['robot_ok']['value'] is False
    assert second['execution_feasibility']['clearance_m']['value']==-.1
    assert second['execution_feasibility']['probability_score']['value'] is None


def test_missing_metrics_stay_null_with_reason_without_safe_defaults():
    result=evaluate_record({'missing_reasons':{'packing_fraction':'not observed'}})
    assert result['legacy_cost']['value'] is None
    assert result['load']['L']['value'] is None
    assert result['stability']['S']['value'] is None
    assert result['space']['packing_fraction']=={'value':None,'reason':'not observed'}
    assert result['execution_feasibility']['robot_ok']['value'] is None


@pytest.mark.parametrize('value',[float('nan'),float('inf'),True,-.1])
def test_bad_raw_geometry_is_rejected(value):
    data=record();data['geometry_features']['void']=value
    with pytest.raises(ValueError):evaluate_record(data)


def test_one_missing_component_prevents_S_and_legacy_only():
    data=record();del data['stability_components']['side']
    out=evaluate_record(data)
    assert out['stability']['S']['value'] is None
    assert out['legacy_cost']['value'] is None
    assert out['space']['legacy_efficiency_cost']['value']==0.
    assert out['load']['L']['value']==1.


def test_boolean_feasibility_is_not_probability():
    data=record();data['execution']['robot_ok']=1
    with pytest.raises(ValueError):evaluate_record(data)
