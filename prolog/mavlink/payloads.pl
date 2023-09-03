/*  File:    mavlink/payloads.pl
    Author:  Roy Ratcliffe
    Created: Sep  3 2023
    Purpose: MAVLink Payloads
*/

:- module(mavlink_payloads,
          []).
:- use_module(endian).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Given a $Msg identifier and a $Payload, the following unifies the
little-endian payload octets with sorted fields by type.

mavlink:message(MessageName, $Msg, _),
    mavlink_sorted_ext_fields(MessageName, Fields),
    phrase(mavlink_payloads:payload(Fields, [], Terms), $Payload).

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

payload([], Terms, Terms) --> [].
payload([H|T], Terms, [Term|Terms_]) -->
    field(H, Term),
    payload(T, Terms, Terms_).

field(FieldName-AtomicType, Term) -->
    { mavlink_type_atom(Type, AtomicType),
      mavlink_type_size(Type, Size),
      Width is Size << 3
    },
    endian(little, Width, Int),
    { Term =.. [FieldName, Int]
    }.
