"""The agility.messages module contains the Message class used to represent
messages received from the robot, as well as a collection of message creation
helpers.

When using the message creation helpers, we recommend using a shortened import
alias for them::

    import agility.messages as msgs
    msg = msgs.ActionGoto(target=msgs.Pose(xy=[10, 0]))

The message creation helpers are not needed to send messages using
:class:`.JsonApi`. All methods accept equivalent JSON-like python objects as
well, and if no fields need to be specified, they will also accept just the
message type as a string::

    # All of the following are equivalent
    await api.send(msgs.ActionStand())
    await api.send(["action-stand", {}])
    await api.send("action-stand")

    # Equivalent ways of specifying a message with explicit field values
    await api.send(msgs.ActionStand(base_pose=msgs.Pose(xyz=[0, 0, 1])))
    await api.send(["action-stand", {"base-pose": {"xyz": [0, 0, 1]}}])
    await api.send(msgs.ActionStand(base_pose={"xyz": [0, 0, 1]}))

If your code editor supports autocompletion and it is correctly configured, the
message creation helpers can help write code more quickly. For example, typing
"msgs.Action" should bring up a tab-complete list of action messages, and when
a specific action is selected your editor may also list the message fields
available as keyword arguments.

When using an older version of this library with a newer version of the
simulator, any newly added message types will not have corresponding message
creation helpers. In this case, you can always just specify the message in
JSON-like format.

The message creation helpers themselves are not listed in this documentation.
Instead, consult the main `JSON API documentation </doc/software/jsonapi.html>`_
for descriptions of each message. The message helpers and their fields are
named after the JSON messages as listed in the main API documentation, but
message names are converted to CamelCase and fields are converted to use
underscores_as_separators instead of dashes, in accordance with Python's syntax
and standard style. Structs and enums also have helpers in the agility.messages
module.

"""

import json
from typing import Any


def _to_json(name: str) -> str:
    """Converts a field name to JSON API convention (dashes)."""
    return name.replace('_', '-')


def _to_py(name: str) -> str:
    """Converts a field name to Python convention (underscores)."""
    return name.replace('-', '_')


def _to_dumpable(value: Any) -> Any:
    """Converts an object to something can be directly serialized by json.dumps.

    Any fields set to the special value OMIT are completely omitted from the
    resulting JSON. Any fields set to None are converted to JSON `null` but are
    included. Field names have underscores converted to dashes.

    """
    if isinstance(value, Struct): # also Message
        fields = {}
        for f in value:
            v = value[f]
            if v is not OMIT:
                fields[_to_json(f)] = _to_dumpable(v)
        if isinstance(value, Message):
            if value._refnum is not None:
                return (value._type, fields, value._refnum)
            else:
                return (value._type, fields)
        else:
            return fields
    elif isinstance(value, list) or isinstance(value, tuple):
        return [_to_dumpable(e) for e in value]
    else:
        return value


def _from_loaded(value: Any) -> Any:
    """Converts an object deserialized by json.loads into a Message/Struct."""
    if isinstance(value, list):
        if (len(value) >= 2 and
            isinstance(value[0], str) and isinstance(value[1], dict)):
            # Treat this as a message
            m = Message(value[0])
            for f in value[1]:
                m[f] = _from_loaded(value[1][f])
            if len(value) >= 3:
                m._refnum = value[2]
            return m
        else:
            # Presumed to be an ordinary homogeneous array
            return [_from_loaded(x) for x in value]
    elif isinstance(value, dict):
        # Treat this as a struct
        s = Struct()
        for f in value:
            s[f] = _from_loaded(value[f])
        return s
    else:
        # Primitive types
        return value


class Struct:
    """Base class for JSON structs.

    Permits accessing fields as attributes instead of just as dictionary
    keys. Structs in incoming messages will be converted to this type, but
    outgoing messages can either be a Struct or an equivalent dict.

    This class can be instantiated directly, but it is generally better to use
    the type-specific subclasses, which verify the keyword arguments passed to
    the function.

    """
    def __init__(self, **kwargs):
        for key, val in kwargs.items():
            self.__dict__[_to_py(key)] = val

    def __iter__(self):
        for f in self.__dict__:
            if f[0] != '_': # Skip internal attributes
                yield f

    def __getitem__(self, key):
        return self.__dict__[_to_py(key)]

    def __setitem__(self, key, value):
        self.__dict__[_to_py(key)] = value

    def __repr__(self):
        return str(_to_dumpable(self))

    def __eq__(self, obj):
        return _to_dumpable(self) == _to_dumpable(obj)

    def __ne__(self, obj):
        return not self == obj


class Message(Struct):
    """Class for JSON messages.

    Permits accessing fields as attributes instead of just as dictionary
    keys. Incoming messages will be converted to this type using the
    :func:`from_json` function, and these messages can be converted to JSON
    using the to_json method.

    This class can be instantiated directly, but it is generally better to use
    the type-specific helper functions, which verify the keyword arguments
    passed to the function.

    """
    def __init__(self, type: str, **kwargs):
        self._type = type
        self._refnum = None # set by _from_loaded, used by JsonApi
        super().__init__(**kwargs)

    @property
    def type(self):
        return self._type

    def to_json(self):
        return json.dumps(_to_dumpable(self))


def from_json(json_data: str) -> Message:
    """Converts a JSON message string into a :class:`Message` object."""
    return _from_loaded(json.loads(json_data))


OMIT = object()
"""Special value used for fields that should be omitted from JSON messages.

   Primitive types follow the standard Python JSON serialization rules for
   conversion into JSON types, with ``None`` being converted into JSON's
   ``null``. Fields that should be entirely left out of a JSON message should
   be set to the special value ``OMIT``. This is the default value of all
   message helper class fields, so it should rarely be necessary to explicitly
   set any field to ``OMIT``.

"""
