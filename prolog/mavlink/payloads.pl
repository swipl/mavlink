/*  File:    mavlink/payloads.pl
    Author:  Roy Ratcliffe
    Created: Sep  3 2023
    Purpose: MAVLink Payloads
*/

:- module(mavlink_payloads,
          [ mavlink_payload//2
          ]).
:- use_module(library(apply)).
:- use_module(library(lists)).
:- use_module(library(mavlink/endian)).
:- use_module(library(mavlink/ieee)).
:- use_module(library(mavlink/types)).

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

%!  mavlink_payload(Msg, Terms)// is semidet.

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

Zero extends the payload octets automatically.

Arrays become list terms. Note, this occurs even for single-item arrays.
The specification allows for this, even though not used in practice.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

payload([], Terms, Terms) --> [].
payload([H|T], Terms, [Term|Terms_]) -->
    field(H, Term),
    payload(T, Terms, Terms_).

field(FieldName-Type, Term, H, T) :-
    Term =.. [FieldName, Value],
    mavlink_type_size(Type, Size),
    (   nonvar(H),
        length(H, Len),
        Len < Size
    ->  Size_ = Len
    ;   Size_ = Size
    ),
    Width is Size_ << 3,
    field(Type, Width, Value, H, T).

field(float, Width, Value) --> float(Width, Value).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Conversion from Int to float may deliver an integer value if a whole
number. This disregards the type, however. Cast the Float result to
floating-point representation therefore regardless using is/2's float/1.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

float(Width, Value) -->
    { var(Value), !
    },
    endian(little, Width, Int),
    { ieee_754_float(Width, Int, Float),
      Value is float(Float)
    }.
float(Width, Value) -->
    { ieee_754_float(Width, Int, Value)
    },
    endian(little, Width, Int).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Appends zeros to Terms0, giving Terms. Can be used to count the
    number of zeros when ZerosLen is unbound albeit more expensively by
    non-deterministic search; take the first solution only.

@arg Terms0 are the terms to append.
@arg ZerosLen is the number of zeros appended.
@arg Terms are the zero-appended terms.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

append_zeros(Terms0, ZerosLen, Terms), var(ZerosLen) =>
    append(Terms0, Zeros, Terms),
    length(Zeros, ZerosLen),
    maplist(=(0), Zeros).
append_zeros(Terms0, ZerosLen, Terms), integer(ZerosLen) =>
    length(Zeros, ZerosLen),
    maplist(=(0), Zeros),
    append(Terms0, Zeros, Terms).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Counts the number of trailing zeros in a list of octets.

@arg Terms is a list of 8-bit bytes, typically but not necessarily.

Given a list of integers at A, the following snippet counts the zeros
then unifies D with the initial sub-list *without* the trailing zeros.

?- A = [1, 0, 2, 0, 3, 0, 0, 0],
    mavlink_payloads:trailing_zeros(A, B),
    length(C, B),
    once(append(D, C, A)).
A = [1, 0, 2, 0, 3, 0, 0, 0],
B = 3,
C = [0, 0, 0],
D = [1, 0, 2, 0, 3].

The final append/3 requires a once/1 in order to cut the final choice
because D is an unbound variable.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

trailing_zeros(Terms, ZerosLen) :- trailing_zeros(Terms, 0, ZerosLen).

trailing_zeros([], ZerosLen, ZerosLen).
trailing_zeros([0|T], ZerosLen0, ZerosLen) :-
    !,
    succ(ZerosLen0, ZerosLen_),
    trailing_zeros(T, ZerosLen_, ZerosLen).
trailing_zeros([_|T], _, ZerosLen) :- trailing_zeros(T, 0, ZerosLen).
