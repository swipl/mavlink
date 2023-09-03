/*  File:    mavlink/types.pl
    Author:  Roy Ratcliffe
    Created: Aug 26 2023
    Purpose: MAVLink Types
*/

:- module(mavlink_types,
          [ mavlink_type_len_atom/3,            % ?Term,?Len,?Atom
            mavlink_type_atom/2,                % ?Term,?Atom
            mavlink_type_atom/3,                % ?Term,?Len,?Atom
            mavlink_type_size/2,                % ?Term,?Size
            mavlink_type_atom_size/2            % ?Atom,?Size
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
type(float) --> "float".
type(double) --> "double".

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

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Unifies a type's Term and Size.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

%!  mavlink_type_size(?Term, ?Size) is nondet.

mavlink_type_size(int(Width), Size) :- width(Width), Size is Width >> 3.
mavlink_type_size(uint(Width), Size) :- width(Width), Size is Width >> 3.
mavlink_type_size(char, 1).
mavlink_type_size(float, 4).
mavlink_type_size(double, 8).

%!  mavlink_type_atom_size(+Atom, ?Size) is semidet.
%!  mavlink_type_atom_size(-Atom, ?Size) is nondet.
%
%   Size of type by Atom.
%
%   The type unifies with the fundamental type _without_ its length when
%   the type specifies an array.

mavlink_type_atom_size(Atom, Size) :-
    nonvar(Atom),
    mavlink_type_len_atom(Type, _, Atom),
    !,
    mavlink_type_size(Type, Size).
mavlink_type_atom_size(Atom, Size) :-
    mavlink_type_atom(Type, Atom),
    mavlink_type_size(Type, Size).
