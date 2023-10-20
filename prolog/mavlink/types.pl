/*  File:    mavlink/types.pl
    Author:  Roy Ratcliffe
    Created: Aug 26 2023
    Purpose: MAVLink Types
*/

:- module(mavlink_types,
          [ mavlink_type_len_atom/3,            % ?Term,?Len,?Atom
            mavlink_type_atom/2,                % ?Term,?Atom
            mavlink_type_atom/3,                % ?Term,?Len,?Atom
            mavlink_type_atom_size/2,           % ?Atom,?Size
            mavlink_basic_size/2,               % ?Basic,?Size
            mavlink_array_len/4,                % ?Array,?Basic,?Size,?Len
            mavlink_array_size/4                % ?Array,?Basic,?Len,?Size
          ]).
:- autoload(library(dcg/basics), [integer//1]).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Converts between Term-Len and Atom. The predicate operates entirely
non-deterministically both forwards _and_ backwards.

Arity-2 helps to determine the basic type either without or with an
array length.

Makes use of the "soft cut."

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

%!  mavlink_type_atom(?Term, ?Len, ?Atom) is det.

mavlink_type_atom(Term, Len, Atom) :-
    mavlink_type_len_atom(Term, Len, Atom) *-> true.
mavlink_type_atom(Term, 1, Atom) :- mavlink_type_atom(Term, Atom).

type(Type, Len) --> type_len(Type, Len), !.
type(Type, 1) --> type(Type).

%!  mavlink_type_len_atom(?Term, ?Len, ?Atom) is nondet.
%
%   True when array type Term and Len match Atom.
%
%   The length of an array must lie between 1 and 255 inclusively. The
%   CRC extra accumulation folds the array length as eight-bit data. A
%   length of zero makes no sense.
%
%   Fails for non-atomic type Atom. Operates non-determistically for
%   variable Type but semi-deterministically for atomic type Atom.
%
%   @arg Atom is the atomic representation of the type Term and its
%   Len.

mavlink_type_len_atom(Term, Len, Atom), var(Atom) =>
    phrase(type_len(Term, Len), Codes),
    atom_codes(Atom, Codes).
mavlink_type_len_atom(Term, Len, Atom), atomic(Atom) =>
    atom_codes(Atom, Codes),
    once(phrase(type_len(Term, Len), Codes)).

type_len(Type, Len) -->
    type(Type),
    "[",
    { between(1, 255, Len)
    },
    integer(Len),
    "]".

%!  mavlink_type_atom(?Term, ?Atom) is nondet.

mavlink_type_atom(Term, Atom), var(Atom) =>
    phrase(type(Term), Codes),
    atom_codes(Atom, Codes).
mavlink_type_atom(Term, Atom), atomic(Atom) =>
    atom_codes(Atom, Codes),
    once(phrase(type(Term), Codes)).

type(int(Width)) -->
    "int",
    { width(Width)
    },
    integer(Width),
    "_t".
type(uint(Width)) --> "u", uint_type(Width).
type(char) --> "char".
type(float(32)) --> "float".
type(float(64)) --> "double".

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Another way to implement this predicate would be to match the Width
    non-deterministically as follows. However, the optional MAVLink version
    integer accepts only the eight-bit integer form, a byte. So the
    following would allow 16, 32- or even 64-bit version integers.

        uint_type(Width) -->
            type(int(Width)),
            (   []
            ;   "_mavlink_version"
            ).

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

uint_type(Width) --> type(int(Width)).
uint_type(8) --> type(int(8)), "_mavlink_version".

width(8).
width(16).
width(32).
width(64).

%!  mavlink_type_atom_size(+Atom, ?Size) is semidet.
%!  mavlink_type_atom_size(-Atom, ?Size) is nondet.
%
%   Size of type by Atom.
%
%   The type unifies with the fundamental type _without_ its length when
%   the type specifies an array. The predicate fails however if the
%   array length exceeds 255; hence the logic does not entirely
%   ignore length but Size becomes the size of the elements.

mavlink_type_atom_size(Atom, Size) :-
    nonvar(Atom),
    mavlink_type_len_atom(Type, _, Atom),
    !,
    mavlink_type_size(Type, Size).
mavlink_type_atom_size(Atom, Size) :-
    mavlink_type_atom(Type, Atom),
    mavlink_type_size(Type, Size).

%!  mavlink_basic_size(?Basic, ?Size) is nondet.
%
%   Unifies a type's basic Type and Size. Aims for simplicity. Type is
%   the basic type *without* a length suffix as in the case for arrays
%   of Type.

mavlink_basic_size(char, 1).
mavlink_basic_size(int8_t, 1).
mavlink_basic_size(uint8_t, 1).
mavlink_basic_size(uint8_t_mavlink_version, 1).
mavlink_basic_size(int16_t, 2).
mavlink_basic_size(uint16_t, 2).
mavlink_basic_size(int32_t, 4).
mavlink_basic_size(uint32_t, 4).
mavlink_basic_size(float, 4).
mavlink_basic_size(int64_t, 8).
mavlink_basic_size(uint64_t, 8).
mavlink_basic_size(double, 8).

%!  mavlink_array_size(?Array, ?Basic, ?Len, ?Size) is nondet.
%
%   True when Array has Basic type of Len items spanning Size octets.

mavlink_array_size(Array, Basic, Len, Size) :-
    mavlink_array_len(Array, Basic, BasicSize, Len),
    between(1, 255, Len),
    Size is BasicSize * Len.

%!  mavlink_array_len(?Array, ?Basic, ?Size, ?Len) is nondet.
%
%   Finds Len of Array type comprising one or more Basic types each of
%   Size octets. Fails for non-Array types.
%
%   @arg Array must be a variable or arity-1 compound.
%   @arg Basic is the basic type atom.
%   @arg Size is the basic size in octets.
%   @arg Len is the array length, between 1 and 255 inclusive.

mavlink_array_len(Array, Basic, Size, Len), compound(Array) =>
    Array =.. [Basic, Len],
    mavlink_basic_size(Basic, Size).
mavlink_array_len(Array, Basic, Size, Len) =>
    mavlink_basic_size(Basic, Size),
    Array =.. [Basic, Len].
