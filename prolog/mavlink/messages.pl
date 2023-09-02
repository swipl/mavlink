/*  File:    mavlink/messages.pl
    Author:  Roy Ratcliffe
    Created: Aug 13 2023
    Purpose: MAVLink Messages

Copyright (c) 2023, Roy Ratcliffe, Northumberland, United Kingdom

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

:- module(mavlink_messages,
          [ mavlink_message_field/2,            % ?MessageName,?FieldName
            mavlink_ext_message_field/2,        % ?MessageName,?FieldName
            mavlink_sorted_fields/2             % +MessageName,-SortedFields
          ]).
:- autoload(library(sort), [predsort/3]).
:- autoload(library(mavlink/types), [mavlink_type_size_atom/2]).

:- ensure_loaded(library(mavlink)).

%!  mavlink_message_field(?MessageName, ?FieldName) is nondet.
%
%   The two-arity mavlink_message_field/2 predicate finds non-extended
%   message fields for a message and field by name. Finds a field for
%   messages without any message extensions or those that have
%   extensions but the field does not belong to the list of extensions.

mavlink_message_field(MessageName, FieldName) :-
    mavlink:message_field(MessageName, FieldName, _, _),
    \+ mavlink_ext_message_field(MessageName, FieldName).

%!  mavlink_ext_message_field(?MessageName, ?FieldName) is nondet.
%
%   An extended message field finds its name in the message's extensions
%   list. Extended fields have both messages with extensions _and_
%   belong to the extensions.

mavlink_ext_message_field(MessageName, FieldName) :-
    mavlink:message_field(MessageName, FieldName, _, _),
    mavlink:message_extensions(MessageName, Extensions),
    memberchk(FieldName, Extensions).

%!  mavlink_sorted_fields(+MessageName, -SortedFields) is semidet.

mavlink_sorted_fields(MessageName, SortedFields) :-
    findall(FieldName-Type,
            (   mavlink:message_field(MessageName, FieldName, Type, _),
                mavlink_message_field(MessageName, FieldName)
            ), Fields),
    predsort(compare_fields, Fields, SortedFields).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Sort the field name-type pairs by their basic type size but what
    happens when two fields have equal type size? Preserve the order in
    that case.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

compare_fields(Order, _FieldName1-Type1, _FieldName2-Type2) :-
    mavlink_type_size_atom(Size1, Type1),
    mavlink_type_size_atom(Size2, Type2),
    compare(Order_, Size2, Size1),
    (   Order_ == (=)
    ->  Order = (<)
    ;   Order = Order_
    ).
