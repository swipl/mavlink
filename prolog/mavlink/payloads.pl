/*  File:    mavlink/payloads.pl
    Author:  Roy Ratcliffe
    Created: Sep  3 2023
    Purpose: MAVLink Payloads
*/

:- module(mavlink_payloads,
          []).
:- use_module(endian).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Decode a heartbeat payload phrase using:

    phrase(mavlink_payload(0, Terms), [0, 0, 0, 0, 6, 8, 192, 4, 3]).

It gives the terms:

    Terms = [ custom_mode(0),
              type(6),
              autopilot(8),
              base_mode(192),
              system_status(4),
              mavlink_version(3)
            ]

The phrasing operates in reverse, of course.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

mavlink_payload(Msg, Terms) -->
    { mavlink:message(MessageName, Msg, _),
      mavlink_sorted_ext_fields(MessageName, Fields)
    },
    payload(Fields, [], Terms).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Given a $Msg identifier and a $Payload from a decoded MAVLink frame, the
following unifies the little-endian payload octets with sorted fields by
type.

mavlink:message(MessageName, $Msg, _),
    mavlink_sorted_ext_fields(MessageName, Fields),
    phrase(mavlink_payloads:payload(Fields, [], Terms), $Payload).

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

payload([], Terms, Terms) --> [].
payload([H|T], Terms, [Term|Terms_]) -->
    field(H, Term),
    payload(T, Terms, Terms_).

field(FieldName-AtomicType, Term) -->
    { Term =.. [FieldName, Int],
      mavlink_type_atom(Type, AtomicType),
      mavlink_type_size(Type, Size),
      Width is Size << 3
    },
    endian(little, Width, Int).
