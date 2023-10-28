/*  File:    mavlink/grammars.pl
    Author:  Roy Ratcliffe
    Created: Oct 28 2023
    Purpose: MAVLink Grammars
*/

:- module(mavlink_grammars,
          [ mavlink_array//1,
            mavlink_array//2,
            mavlink_basic//1
          ]).
:- autoload(library(dcg/basics), [integer/3, csym/3]).

%!  mavlink_array(?Array)// is semidet.
%!  mavlink_array(?Basic, ?Len)// is semidet.
%
%   True when a basic-length phrase matches Basic type of Len items.
%   Fails for non-array type phrases.
%
%   Array types _should_ fail for `uint8_t_mavlink_version` basic types.
%   The MAVLink version must be a single basic field, not an array.
%
%   @arg Array is an arity-1 compound where the functor name matches the
%   Basic type and the argument is an array Len integer.
%
%   @arg Basic is the base type atom.
%
%   @arg Len is the integer length of the array between 1 and 255
%   inclusively.

mavlink_array(Array) -->
    { nonvar(Array), !, Array =.. [Basic, Len]
    },
    mavlink_array(Basic, Len).
mavlink_array(Array) -->
    mavlink_array(Basic, Len),
    { Array =.. [Basic, Len]
    }.

mavlink_array(Basic, Len) -->
    mavlink_basic(Basic), "[", integer(Len), "]".

%!  mavlink_basic(?Basic)// is semidet.
%
%   Implements the MAVLink basic type grammar. Allows for *any* C-style
%   type specifier without strict checking for validity of type; valid
%   checks come later when incoming message fields unify with message
%   frame octets. Allow flexibility at this stage.
%
%   Utilises the csym//1 grammar. It has one problem when Basic is
%   non-variable. The implementation leaves a choicepoint which blows
%   up the trail stack. Fix this issue by preempting the non-variable
%   choice and introducing a cut.

mavlink_basic(Basic) -->
    { nonvar(Basic), !
    },
    csym(Basic), !.
mavlink_basic(Basic) --> csym(Basic).
