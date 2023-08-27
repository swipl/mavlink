/*  File:    mavlink/extras.pl
    Author:  Roy Ratcliffe
    Created: Aug 26 2023
    Purpose: MAVLink Extras
*/

:- module(mavlink_extras,
          [ mavlink_extra/2                     % +MsgId,-Extra
          ]).
:- autoload(library(apply), [foldl/4]).
:- autoload(library(sort), [predsort/3]).
:- use_module(messages).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Computes the Extra byte for a given MsgId. The first argument
MsgId is the message identifier, not the message name.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

:- table mavlink_extra/2.

%!  mavlink_extra(+MsgId, -Extra) is semidet.

mavlink_extra(MsgId, Extra) :-
    crc_16_mcrf4xx(Check0),
    mavlink:message(MessageName, MsgId, _),
    crc_16_mcrf4xx(Check0, MessageName, Check1),
    crc_16_mcrf4xx(Check1, 0' , Check2),
    sorted_fields(MessageName, SortedFields0),
    predsort(pred, SortedFields0, SortedFields),
    foldl(mavlink_extra_, SortedFields, Check2, Check3),
    Extra is (Check3 /\ 16'FF) xor (Check3 >> 8).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Note, there are two version of a `uint8_t` term: an additional
    MAVLink version variety. It occurs only once in the `HEARTBEAT`
    message. Ignore the latter alternative for unsigned 8-bit integers.
    Otherwise a choice point appears. Hence the two once/1 calls.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

mavlink_extra_(FieldName-Type, Check0, Check) :-
    once(mavlink_type_atom(Term, _, Type)),
    once(mavlink_type_atom(Term, Atom)),
    crc_16_mcrf4xx(Check0, Atom, Check1),
    crc_16_mcrf4xx(Check1, 0' , Check2),
    crc_16_mcrf4xx(Check2, FieldName, Check3),
    crc_16_mcrf4xx(Check3, 0' , Check4),
    (   mavlink_type_length_atom(_, Length, Type)
    ->  crc_16_mcrf4xx(Check4, Length, Check)
    ;   Check = Check4
    ).

sorted_fields(MessageName, Fields) :-
    findall(FieldName-Type,
            (   mavlink:message_field(MessageName, FieldName, Type, _),
                mavlink_message_field(MessageName, FieldName)
            ), Fields).

pred(Order, _FieldName1-Type1, _FieldName2-Type2) :-
    type_size(Type1, Size1),
    type_size(Type2, Size2),
    compare(Order_, Size2, Size1),
    (   Order_ == (=)
    ->  Order = (<)
    ;   Order = Order_
    ).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Size of type by Atom.

    Type_ unifies with the base type _without_ its length when Type
    specifies an array type.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

type_size(Atom, Size) :-
    mavlink_type_length_atom(Type, _, Atom),
    !,
    mavlink_types:mavlink_type_size(Type, Size).
type_size(Atom, Size) :-
    mavlink_type_atom(Type, Atom),
    mavlink_types:mavlink_type_size(Type, Size).
