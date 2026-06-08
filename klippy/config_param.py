from __future__ import annotations

from enum import Enum
from typing import Generic, TypeVar

from attr import dataclass, field

from klippy.configfile import ConfigWrapper
from klippy.gcode import GCodeCommand
from klippy.mathutil import safe_float


### TODO
# * support multiple separators on the list type
# * support structured maps with different separators
# * support a required count on separated lists
# * use gcode specific command for choice

T = TypeVar("T")
E = TypeVar("E")
NumT = TypeVar("NumT", int, float)


# The get() method has 2 variants, one that returns None and the other that
# raises an exception if no value can be resolved. Both delegate to _resolve(),
# which is supplied by the storage/validation axis (_BasicParam).
@dataclass
class _AbstractParamGetter(Generic[T]):
    name: str

    def resolve(self, gcmd: GCodeCommand | None) -> T | None: ...

    # Full resolution including any default. For nullable params this is just
    # resolve(); _ParamGetter overrides it to fall back to its default.
    def resolve_or_default(self, gcmd: GCodeCommand | None) -> T | None:
        return self.resolve(gcmd)


@dataclass
class _NullableParamGetter(_AbstractParamGetter[T]):
    def get(self, gcmd: GCodeCommand | None = None) -> T | None:
        return self.resolve(gcmd)


@dataclass
class _ParamGetter(_AbstractParamGetter[T]):
    # default can be a plain value or another param of the same type. In the
    # latter case the default is resolved at runtime by delegating to that
    # param's own resolution chain (config value, gcode override, default).
    # Nullable params have no default: assigning one means the value can never
    # resolve to None, so the non-nullable variant should be used instead.
    default: T | _AbstractParamGetter[T] | None = None

    def get(self, gcmd: GCodeCommand | None = None) -> T:
        value = self.resolve_or_default(gcmd)
        if value is None:
            raise ValueError("Parameter '%s' has no value" % (self.name,))
        return value

    def resolve_or_default(self, gcmd: GCodeCommand | None) -> T | None:
        value = self.resolve(gcmd)
        if value is None:
            if isinstance(self.default, _AbstractParamGetter):
                value = self.default.resolve_or_default(gcmd)
            else:
                value = self.default
        return value


# Source: where a param's value is allowed to come from. This does not
# affect the static return type of get(), so it is a runtime field rather than
# a separate type in the class hierarchy.
class Source(Enum):
    BOTH = "both"  # config file value, overridable by a gcode command
    CONFIG = "config"  # config file only; gcode commands are ignored
    GCODE = "gcode"  # gcode command only; never read from the config file


# Storage + validation axis. config() and _resolve() both funnel raw values
# through _convert() + _validate() so the same checks apply regardless of
# whether the value came from the config file or a gcode command.
@dataclass
class _BaseParam(Generic[T]):
    name: str
    gcode_name: str | None = None
    required_in_config: bool = False
    source: Source = Source.BOTH
    _config_value: T | None = field(init=False, default=None)

    def __attrs_post_init__(self):
        if self.source is Source.GCODE and self.required_in_config:
            raise ValueError(
                "Parameter '%s' cannot be required_in_config with a "
                "gcode-only source" % (self.name,)
            )

    def config(self, config: ConfigWrapper):
        if self.source is Source.GCODE:
            return
        raw = config.get(self.name, None)
        if raw is None:
            if self.required_in_config:
                raise config.error(
                    "Option '%s' must be specified" % (self.name,)
                )
            return
        self._config_value = self._validate(self._convert(raw))

    def resolve(self, gcmd: GCodeCommand | None) -> T | None:
        if self.source is not Source.CONFIG and gcmd is not None:
            override = self._read_gcode(gcmd)
            if override is not None:
                return override
        return self._config_value

    def _get_gcode_name(self):
        if self.gcode_name is not None:
            return self.gcode_name.upper()
        return self.name.upper()

    def _read_gcode(self, gcmd: GCodeCommand) -> T | None:
        raw = gcmd.get(self._get_gcode_name(), None)
        if raw is None:
            return None
        return self._validate(self._convert(raw))

    def _convert(self, raw: str) -> T:
        raise NotImplementedError

    def _validate(self, value: T) -> T:
        return value


def _check_numeric(name, value, min_val, max_val, above, below):
    if min_val is not None and value < min_val:
        raise ValueError("'%s' must have minimum of %s" % (name, min_val))
    if max_val is not None and value > max_val:
        raise ValueError("'%s' must have maximum of %s" % (name, max_val))
    if above is not None and value <= above:
        raise ValueError("'%s' must be above %s" % (name, above))
    if below is not None and value >= below:
        raise ValueError("'%s' must be below %s" % (name, below))
    return value


@dataclass
class _NumericParam(_BaseParam[NumT]):
    above: NumT | None = None
    below: NumT | None = None
    min_val: NumT | None = None
    max_val: NumT | None = None

    def _validate(self, value: NumT) -> NumT:
        return _check_numeric(
            self.name,
            value,
            self.min_val,
            self.max_val,
            self.above,
            self.below,
        )


# Element axis: each base owns the str -> T conversion, defined once and shared
# by the required and nullable variants.
def _to_bool(raw: str) -> bool:
    value = raw.strip().lower()
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off", ""):
        return False
    raise ValueError("Cannot parse boolean from '%s'" % (raw,))


@dataclass
class _BoolParam(_BaseParam[bool]):
    def _convert(self, raw: str) -> bool:
        return _to_bool(raw)

    def _read_gcode(self, gcmd: GCodeCommand) -> bool | None:
        if gcmd.get(self._get_gcode_name(), None) is None:
            return None
        return gcmd.has_flag(self._get_gcode_name())


@dataclass
class _StringParam(_BaseParam[str]):
    def _convert(self, raw: str) -> str:
        return raw


@dataclass
class _IntParam(_NumericParam[int]):
    def _convert(self, raw: str) -> int:
        return int(raw)


@dataclass
class _FloatParam(_NumericParam[float]):
    def _convert(self, raw: str) -> float:
        return safe_float(raw)


# Concrete types = element axis x nullability axis.
@dataclass
class BoolParam(_BoolParam, _ParamGetter[bool]): ...


@dataclass
class NullableBoolParam(_BoolParam, _NullableParamGetter[bool]): ...


@dataclass
class StringParam(_StringParam, _ParamGetter[str]): ...


@dataclass
class NullableStringParam(_StringParam, _NullableParamGetter[str]): ...


@dataclass
class IntParam(_IntParam, _ParamGetter[int]): ...


@dataclass
class NullableIntParam(_IntParam, _NullableParamGetter[int]): ...


@dataclass
class FloatParam(_FloatParam, _ParamGetter[float]): ...


@dataclass
class NullableFloatParam(_FloatParam, _NullableParamGetter[float]): ...


# Choice type. Maps a raw value to one of a fixed set of choices. `choices` may
# be a dict (raw key -> resolved value) or a list (treated as an identity map),
# matching ConfigWrapper.getchoice(). When the keys are ints, the raw value is
# parsed as an int before lookup. The default (if any) is a *key* into choices,
# not a resolved value: it is validated and mapped just like a config value.
@dataclass
class _ChoiceElem(_BaseParam[T]):
    choices: dict | list = field(factory=dict)

    def _choices_map(self) -> dict:
        if isinstance(self.choices, list):
            return {c: c for c in self.choices}
        return self.choices

    def _lookup(self, key, raw) -> T:
        choices = self._choices_map()
        if key not in choices:
            raise ValueError(
                "Choice '%s' for '%s' is not a valid choice" % (raw, self.name)
            )
        return choices[key]

    def _convert(self, raw: str) -> T:
        choices = self._choices_map()
        if choices and isinstance(next(iter(choices)), int):
            key = int(raw)
        else:
            key = raw
        return self._lookup(key, raw)


@dataclass
class ChoiceParam(_ChoiceElem[T], _ParamGetter[T]):
    # default is a key into choices (matching ConfigWrapper.getchoice), so it is
    # mapped through _lookup rather than returned directly. A param chained as
    # the default resolves to a value and is returned directly.
    def resolve_or_default(self, gcmd: GCodeCommand | None) -> T | None:
        value = self.resolve(gcmd)
        if value is None:
            default = self.default
            if isinstance(default, _AbstractParamGetter):
                value = default.resolve_or_default(gcmd)
            elif default is not None:
                value = self._lookup(default, default)
        return value


@dataclass
class NullableChoiceParam(_ChoiceElem[T], _NullableParamGetter[T]): ...


# List element types. Splits the raw string on `sep` and converts each element.
# An empty (but present) value resolves to an empty list, matching the behavior
# of ConfigWrapper.getlists(); an omitted option stays None (see _BasicParam).
@dataclass
class _ListParam(_BaseParam[list[E]]):
    sep: str = ","

    def _convert(self, raw: str) -> list[E]:
        if len(raw.strip()) == 0:
            return []
        return [self._convert_elem(p.strip()) for p in raw.split(self.sep)]

    def _convert_elem(self, raw: str) -> E:
        raise NotImplementedError


@dataclass
class _NumericListParam(_ListParam[NumT]):
    above: NumT | None = None
    below: NumT | None = None
    min_val: NumT | None = None
    max_val: NumT | None = None

    def _validate(self, value: list[NumT]) -> list[NumT]:
        for elem in value:
            _check_numeric(
                self.name,
                elem,
                self.min_val,
                self.max_val,
                self.above,
                self.below,
            )
        return value


@dataclass
class _StrListElem(_ListParam[str]):
    def _convert_elem(self, raw: str) -> str:
        return raw


@dataclass
class _IntListElem(_NumericListParam[int]):
    def _convert_elem(self, raw: str) -> int:
        return int(raw)


@dataclass
class _FloatListElem(_NumericListParam[float]):
    def _convert_elem(self, raw: str) -> float:
        return safe_float(raw)


@dataclass
class StringListParam(_StrListElem, _ParamGetter[list[str]]): ...


@dataclass
class NullableStringListParam(
    _StrListElem, _NullableParamGetter[list[str]]
): ...


@dataclass
class IntListParam(_IntListElem, _ParamGetter[list[int]]): ...


@dataclass
class NullableIntListParam(_IntListElem, _NullableParamGetter[list[int]]): ...


@dataclass
class FloatListParam(_FloatListElem, _ParamGetter[list[float]]): ...


@dataclass
class NullableFloatListParam(
    _FloatListElem, _NullableParamGetter[list[float]]
): ...
