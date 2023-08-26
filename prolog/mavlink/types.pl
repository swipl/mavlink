/*  File:    mavlink/types.pl
    Author:  Roy Ratcliffe
    Created: Aug 26 2023
    Purpose: MAVLink Types
*/

:- module(mavlink_types,
          [ mavlink_type_length_atom/3,         % ?Term,?Length,?Atom
            mavlink_type_atom/2,                % ?Term,?Atom
            mavlink_type_atom/3,                % ?Term,?Length,?Atom
            mavlink_type_size/2                 % ?Term,?Size
          ]).
:- autoload(library(dcg/basics), [integer//1]).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

The predicate operates entirely non-deterministically both forwards
_and_ backwards.

Arity-2 helps to determine the basic type either without or with an
array length.

Makes use of the "soft cut."

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

%!  mavlink_type_atom(?Term, ?Length, ?Atom) is det.

mavlink_type_atom(Term, Length, Atom) :-
    mavlink_type_length_atom(Term, Length, Atom) *-> true.
mavlink_type_atom(Term, 1, Atom) :- mavlink_type_atom(Term, Atom).

type(Type, Length) --> type_length(Type, Length), !.
type(Type, 1) --> type(Type).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

True when array Type and Length match Codes.

The length of an array must lie between 1 and 255 inclusively. The CRC
extra accumulation folds the array length as eight bit data. A length of
zero makes no sense.

Fails for non-atomic Type. Operates non-determistically for variable
Type but semi-deterministically for atomic Type.

@arg Atom is the atomic representation of the type Term and its Length.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

%!  mavlink_type_length_atom(?Term, ?Length, ?Atom) is nondet.

mavlink_type_length_atom(Term, Length, Atom), var(Atom) =>
    phrase(type_length(Term, Length), Codes),
    atom_codes(Atom, Codes).
mavlink_type_length_atom(Term, Length, Atom), atomic(Atom) =>
    atom_codes(Atom, Codes),
    once(phrase(type_length(Term, Length), Codes)).

type_length(Type, Length) -->
    type(Type),
    "[",
    { between(1, 255, Length)
    },
    integer(Length),
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

%!  mavlink_type_size(?Term, ?Size) is nondet.

mavlink_type_size(int(Width), Size) :- width(Width), Size is Width >> 3.
mavlink_type_size(uint(Width), Size) :- width(Width), Size is Width >> 3.
mavlink_type_size(char, 1).
mavlink_type_size(float, 4).
mavlink_type_size(double, 8).
