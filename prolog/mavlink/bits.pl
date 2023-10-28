/*  File:    canny/bits.pl
    Author:  Roy Ratcliffe
    Created: Aug 15 2022
    Purpose: Canny Bits

Copyright (c) 2022, Roy Ratcliffe, Northumberland, United Kingdom

Permission is hereby granted, free of charge,  to any person obtaining a
copy  of  this  software  and    associated   documentation  files  (the
"Software"), to deal in  the   Software  without  restriction, including
without limitation the rights to  use,   copy,  modify,  merge, publish,
distribute, sublicense, and/or sell  copies  of   the  Software,  and to
permit persons to whom the Software is   furnished  to do so, subject to
the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT  WARRANTY OF ANY KIND, EXPRESS
OR  IMPLIED,  INCLUDING  BUT  NOT   LIMITED    TO   THE   WARRANTIES  OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR   PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS  OR   COPYRIGHT  HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY,  WHETHER   IN  AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM,  OUT  OF   OR  IN  CONNECTION  WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

:- module(mavlink_bits,
          [ bits/5,
            bits/4,
            bits/3,
            bit_fields/3,                       % +Fields,+Shift,+Int
            bit_fields/4,                       % +Fields,+Shift,+Int0,-Int
            rbit/3                              % +Shift:integer,+Int,?Reverse
          ]).

:- use_module(library(clpfd)).

%!  bits(+Shift, +Width, ?Word, ?Bits, ?Rest) is semidet.
%!  bits(+ShiftWidthPair, ?Word, ?Bits, ?Rest) is semidet.
%!  bits(+ShiftWidthPair, ?Word, ?Bits) is semidet.
%
%   Unifies Bits within a Word using Shift  and Width. All arguments are
%   integers treated as words of arbitrary bit-width.
%
%   The implementation uses relational integer arithmetic, i.e. CLP(FD).
%   Hence allows for forward and backward   transformations from Word to
%   Bits and vice versa. Integer Word applies a Shift and bit Width mask
%   to integer Bits. Bits is always   a  smaller integer. Decomposes the
%   problem  into  shifting  and  masking.    Treats   these  operations
%   separately.
%
%   @arg Width of Bits from  Word  after   Shift.  Width  of zero always
%   fails.

bits(Shift, Width, Word, Bits, Rest) :-
    Mask #= (1 << Width - 1) << Shift,
    Bits0 #= Word /\ Mask,
    Rest #= Word xor Bits0,
    Word #= Bits0 /\ Mask \/ Rest,
    Bits #= Bits0 // (1 << Shift),
    Bits0 #= Bits << Shift.

bits(Shift-Width, Word, Bits, Rest) :- bits(Shift, Width, Word, Bits, Rest).

bits(Shift-Width, Word, Bits) :- bits(Shift, Width, Word, Bits, _Rest).

%!  bit_fields(+Fields:list, +Shift:integer, +Int:integer) is semidet.
%!  bit_fields(+Fields:list, +Shift:integer, +Int0:integer,
%!  -Int:integer) is semidet.
%
%   Two versions of the predicate unify `Value`:`Width` bit fields with
%   integers. The arity-3 version requires a bound Int from which to
%   find unbound (or bound) values in the Fields; used to extract values
%   from integers else check values semi-deterministically. The arity-4
%   version of the predicate accumulates bit-field values by OR-wise
%   merging shifted bits from Int0 to Int.
%
%   The predicates are semi-deterministic. They can fail. Failure occurs
%   when the bit-field `Width` integers do *not* sum to Shift.
%
%   @arg Fields is a list of value and width terms of the form
%   `Value:Width` where `Width` is an integer; `Value` is either a
%   variable or an integer.
%
%   @arg Shift is an integer number of total bits, e.g. 8 for eight-bit
%   bytes, 16 for sixteen-bit words and so on.

bit_fields([], 0, _Int).
bit_fields([Value:Width|Rest], Shift, Int) :-
    Shift_ is Shift - Width,
    Value is (Int >> Shift_) /\ ((1 << Width) - 1),
    bit_fields(Rest, Shift_, Int).

bit_fields([], 0, Int, Int).
bit_fields([Value:Width|Rest], Shift, Int0, Int) :-
    Shift_ is Shift - Width,
    Int_ is Int0 \/ ((Value /\ ((1 << Width) - 1)) << Shift_),
    bit_fields(Rest, Shift_, Int_, Int).

%!  rbit(+Shift:integer, +Int:integer, ?Reverse:integer) is semidet.
%
%   Bit reversal over a given span of bits. The reverse bits equal the
%   mirror image of the original; integer $1$ reversed in 16 bits
%   becomes $8000_{16}$ for instance.
%
%   Arity-3 `rbit/3` predicate throws away the residual. Any bit values
%   lying outside the shifting span disappear; they do not appear in the
%   residual and the predicate discards them. The order of the sub-terms
%   is not very important, except for failures. Placing `succ` first
%   ensures that recursive shifting fails if `Shift` is not a positive
%   integer; it triggers an exception if not actually an integer.

rbit(Shift, Int, Reverse) :- rbit(Shift, Int, 0, _Int, Reverse).

rbit(0, Int, Reverse, Int, Reverse) :- !.
rbit(Shift, Int0, Reverse0, Int, Reverse) :-
    succ(Shift_, Shift),
    Reverse_ is (Reverse0 << 1) \/ (Int0 /\ 1),
    Int_ is Int0 >> 1,
    rbit(Shift_, Int_, Reverse_, Int, Reverse).
