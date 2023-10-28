/*  File:    mavlink/extras.pl
    Author:  Roy Ratcliffe
    Created: Aug 26 2023
    Purpose: MAVLink Extras
*/

:- module(mavlink_extras,
          [ mavlink_extra/2                     % ?Msg,?Extra
          ]).
:- autoload(library(apply), [foldl/4]).
:- autoload(library(mavlink/messages), [mavlink_sorted_fields/2]).
:- autoload(library(mavlink/crc_16_mcrf4xx),
            [crc_16_mcrf4xx/1, crc_16_mcrf4xx/3]).
:- use_module(types).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Computes the Extra byte for a given Msg. The first argument
Msg is the message identifier, not the message name.

The predicate is tabled. Abolish the table if the underlying `mavlink`
module predicates change field names or types on which extra CRC byte
calculations depend.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

:- table mavlink_extra/2.

%!  mavlink_extra(?Msg, ?Extra) is nondet.

mavlink_extra(Msg, Extra) :-
    crc_16_mcrf4xx(Check0),
    mavlink:message(MessageName, Msg, _),
    crc(Check0, MessageName, Check1),
    mavlink_sorted_fields(MessageName, SortedFields),
    foldl(mavlink_extra_, SortedFields, Check1, Check2),
    Extra is (Check2 >> 8) xor (Check2 /\ 16'FF).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Note, there are two version of a `uint8_t` term: an additional
    MAVLink version variety. It occurs only once in the `HEARTBEAT`
    message. Ignore the latter alternative for unsigned 8-bit integers.
    Otherwise a choice point appears. Hence the two once/1 calls.

The Type is an atom. Convert this to its fundamental type Term without any
length component. Next, convert the fundamental Term back to an Atom. These two
steps strip away any array specification.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

mavlink_extra_(FieldName-Type, Check0, Check) :-
    once(mavlink_type_atom(Term, _, Type)),
    once(mavlink_type_atom(Term, Atom)),
    crc(Check0, Atom, Check1),
    crc(Check1, FieldName, Check2),
    (   mavlink_type_len_atom(_, Len, Type)
    ->  crc_16_mcrf4xx(Check2, Len, Check)
    ;   Check = Check2
    ).

crc(Check0, Atom, Check) :-
    crc_16_mcrf4xx(Check0, Atom, Check1),
    crc_16_mcrf4xx(Check1, 0' , Check).
