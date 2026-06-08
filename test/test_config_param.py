import pytest

from klippy import config_param, gcode

# --- Test doubles -----------------------------------------------------------


class FakeConfig:
    """Minimal stand-in for ConfigWrapper used by _BasicParam.config()."""

    error = ValueError

    def __init__(self, values=None):
        self._values = values or {}

    def get(self, option, default):
        return self._values.get(option, default)


class DummyGCode:
    def respond_info(self, msg):
        pass

    def respond_raw(self, msg):
        pass


def make_gcmd(params):
    return gcode.GCodeCommand(DummyGCode(), "TEST", "TEST", params, False)


def configure(param, **values):
    param.config(FakeConfig(values))
    return param


# --- Conversion + resolution from config ------------------------------------


def test_int_param_reads_config_value():
    p = configure(config_param.IntParam("speed"), speed="42")
    assert p.get() == 42


def test_float_param_reads_config_value():
    p = configure(config_param.FloatParam("ratio"), ratio="1.5")
    assert p.get() == 1.5


def test_string_param_reads_config_value():
    p = configure(config_param.StringParam("mode"), mode="fast")
    assert p.get() == "fast"


@pytest.mark.parametrize("raw", ["true", "1", "yes", "on"])
def test_bool_param_true_values(raw):
    p = configure(config_param.BoolParam("flag"), flag=raw)
    assert p.get() is True


@pytest.mark.parametrize("raw", ["false", "0", "no", "off", ""])
def test_bool_param_false_values(raw):
    p = configure(config_param.BoolParam("flag"), flag=raw)
    assert p.get() is False


def test_bool_param_invalid_value_raises():
    with pytest.raises(ValueError):
        configure(config_param.BoolParam("flag"), flag="maybe")


# --- Numeric validation -----------------------------------------------------


def test_numeric_min_val_rejected():
    with pytest.raises(ValueError, match="minimum"):
        configure(config_param.IntParam("speed", min_val=10), speed="5")


def test_numeric_max_val_rejected():
    with pytest.raises(ValueError, match="maximum"):
        configure(config_param.IntParam("speed", max_val=10), speed="15")


def test_numeric_above_rejected():
    with pytest.raises(ValueError, match="above"):
        configure(config_param.FloatParam("v", above=0.0), v="0")


def test_numeric_below_rejected():
    with pytest.raises(ValueError, match="below"):
        configure(config_param.FloatParam("v", below=10.0), v="10")


def test_numeric_within_bounds_accepted():
    p = configure(
        config_param.IntParam("speed", min_val=0, max_val=100), speed="50"
    )
    assert p.get() == 50


# --- Defaults ---------------------------------------------------------------


def test_default_used_when_config_absent():
    p = configure(config_param.IntParam("speed", default=99))
    assert p.get() == 99


def test_config_value_overrides_default():
    p = configure(config_param.IntParam("speed", default=99), speed="7")
    assert p.get() == 7


def test_get_raises_when_no_value_and_no_default():
    p = configure(config_param.IntParam("speed"))
    with pytest.raises(ValueError, match="has no value"):
        p.get()


def test_chained_param_default():
    fallback = configure(config_param.IntParam("fallback"), fallback="3")
    p = configure(config_param.IntParam("speed", default=fallback))
    assert p.get() == 3


def test_nullable_param_returns_none_when_absent():
    p = configure(config_param.NullableIntParam("speed"))
    assert p.get() is None


# --- Required in config ------------------------------------------------------


def test_required_in_config_raises_when_absent():
    with pytest.raises(ValueError, match="must be specified"):
        configure(config_param.IntParam("speed", required_in_config=True))


def test_required_in_config_with_gcode_source_raises():
    with pytest.raises(ValueError, match="cannot be required_in_config"):
        config_param.IntParam(
            "speed",
            required_in_config=True,
            source=config_param.Source.GCODE,
        )


# --- Source axis ------------------------------------------------------------


def test_source_config_ignores_gcode_override():
    p = configure(
        config_param.IntParam("speed", source=config_param.Source.CONFIG),
        speed="10",
    )
    gcmd = make_gcmd({"SPEED": "20"})
    assert p.get(gcmd) == 10


def test_source_gcode_ignores_config_value():
    p = configure(
        config_param.IntParam("speed", source=config_param.Source.GCODE),
        speed="10",
    )
    assert p.resolve(None) is None
    gcmd = make_gcmd({"SPEED": "20"})
    assert p.get(gcmd) == 20


def test_source_both_gcode_overrides_config():
    p = configure(config_param.IntParam("speed"), speed="10")
    gcmd = make_gcmd({"SPEED": "20"})
    assert p.get(gcmd) == 20


def test_gcode_name_overrides_lookup_key():
    p = configure(config_param.IntParam("speed", gcode_name="S"), speed="10")
    gcmd = make_gcmd({"S": "20"})
    assert p.get(gcmd) == 20


def test_gcode_override_is_validated():
    p = configure(config_param.IntParam("speed", max_val=100), speed="10")
    gcmd = make_gcmd({"SPEED": "200"})
    with pytest.raises(ValueError, match="maximum"):
        p.get(gcmd)


def test_gcode_lookup_upper_cases_param_name():
    # GCodeDispatch always upper-cases param keys, so the param name is
    # upper-cased before the gcode lookup to match.
    p = configure(config_param.IntParam("speed"), speed="10")
    gcmd = make_gcmd({"SPEED": "20"})
    assert p.get(gcmd) == 20


# --- Bool gcode reading -----------------------------------------------------


def test_bool_param_reads_gcode_flag():
    p = configure(config_param.BoolParam("flag"), flag="false")
    gcmd = make_gcmd({"FLAG": "true"})
    assert p.get(gcmd) is True


# --- Choice axis ------------------------------------------------------------


def test_choice_param_list_choices():
    p = configure(
        config_param.ChoiceParam("mode", choices=["a", "b", "c"]), mode="b"
    )
    assert p.get() == "b"


def test_choice_param_dict_maps_value():
    p = configure(
        config_param.ChoiceParam("mode", choices={"lo": 1, "hi": 2}),
        mode="hi",
    )
    assert p.get() == 2


def test_choice_param_int_keys():
    p = configure(
        config_param.ChoiceParam("mode", choices={1: "one", 2: "two"}),
        mode="2",
    )
    assert p.get() == "two"


def test_choice_param_invalid_choice_raises():
    with pytest.raises(ValueError, match="not a valid choice"):
        configure(
            config_param.ChoiceParam("mode", choices=["a", "b"]), mode="z"
        )


def test_choice_param_default_is_mapped():
    p = configure(
        config_param.ChoiceParam(
            "mode", choices={"lo": 1, "hi": 2}, default="lo"
        )
    )
    assert p.get() == 1


# --- List axis --------------------------------------------------------------


def test_string_list_param():
    p = configure(config_param.StringListParam("names"), names="a, b ,c")
    assert p.get() == ["a", "b", "c"]


def test_int_list_param():
    p = configure(config_param.IntListParam("nums"), nums="1, 2, 3")
    assert p.get() == [1, 2, 3]


def test_float_list_param():
    p = configure(config_param.FloatListParam("nums"), nums="1.0, 2.5")
    assert p.get() == [1.0, 2.5]


def test_list_param_empty_value_is_empty_list():
    p = configure(config_param.StringListParam("names"), names="")
    assert p.get() == []


def test_list_param_custom_separator():
    p = configure(config_param.StringListParam("names", sep=":"), names="a:b:c")
    assert p.get() == ["a", "b", "c"]


def test_numeric_list_validation():
    with pytest.raises(ValueError, match="maximum"):
        configure(config_param.IntListParam("nums", max_val=5), nums="1, 9")
