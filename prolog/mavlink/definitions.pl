/*  File:    mavlink/definitions.pl
    Author:  Roy Ratcliffe
    Created: Aug 12 2023
    Purpose: MAVLink Definitions

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

:- module(mavlink_definitions,
          [ mavlink_definitions_r/2,            % +Base,-Mavlinks:list
            mavlink_definitions/2               % +Base,-Mavlink
          ]).
:- autoload(library(filesex), [directory_file_path/3]).
:- autoload(library(lists), [reverse/2, subtract/3, append/3]).
:- autoload(library(pairs), [pairs_keys/2]).
:- autoload(library(sgml), [load_structure/3]).
:- autoload(library(url), [parse_url/2]).
:- autoload(library(xpath), [xpath/3, xpath_chk/3]).
:- autoload(library(option), [merge_options/3]).

:- use_module(library(xpath)).

%!  mavlink_definitions_r(+Base, -Mavlinks:list) is det.
%
%   Recursively downloads multiple XML elements containing MAVLink
%   message definitions.
%
%   Searches through the included graph of message definitions.
%   Recursively accumulates base-element pairs for a given base, e.g.
%   `all`, along with all its included message definitions, eliminating
%   duplicated includes.
%
%   Things to note:
%
%       * The inner predicate accumulates the base-element pairs in
%       reverse order by pushing the pairs in reverse order. A new
%       message definition pushes to the head of the accumulated list.
%       The outer predicate reverses the accumulator in order to reflect
%       the order of inclusion. The top-level definitions join the list
%       first.
%
%       * The accumulator removes definitions already included as well
%       as self-inclusions if any. For each list of included bases, the
%       logic subtracts (a) the present and outstanding bases and (b)
%       the already accumulated bases. It assumes that the includes do
%       *not* carry duplicates.
%
%       * The result is a list of base-element pairs for each set of
%       `mavlink` message definitions.

mavlink_definitions_r(H, Mavlinks) :-
    mavlink_definitions_r_([H], [], Mavlinks_),
    reverse(Mavlinks_, Mavlinks).

mavlink_definitions_r_([], Acc, Acc).
mavlink_definitions_r_([H|T], Acc0, Acc) :-
    mavlink_definitions(H, Mavlink),
    mavlink_includes(Mavlink, Bases),
    pairs_keys(Acc0, Keys),
    subtract(Bases, [H|T], Bases_),
    subtract(Bases_, Keys, Bases__),
    append(T, Bases__, T_),
    mavlink_definitions_r_(T_, [H-Mavlink|Acc0], Acc).

mavlink_includes(Mavlink, Bases) :-
    findall(Base, mavlink_include(Mavlink, Base), Bases).

mavlink_include(Mavlink, Base) :-
    xpath(Mavlink, include, element(_, _, [Include])),
    file_name_extension(Base, xml, Include).

%!  mavlink_definitions(+Base, -Mavlink) is semidet.
%
%   Downloads MAVLink message definitions from GitHub.
%
%   Loads an arbitrary MAVLink message definition structure where the
%   first `Base` argument specifies `standard` or some other message
%   set; the `xml` extension automatically applies. The predicate
%   requires Internet access to GitHub, naturally. It operates
%   semi-deterministically, not unifying with _every_ `mavlink` element
%   within the definition---with a trailing choice point, as would
%   happen if the predicate tried to find multiple elements. Normally,
%   one definition has one element. The rule unifies with just one by
%   finding the first element; it fails if more than one because only
%   one allowed. Using `xpath_chk/3` instead of `xpath/3` results in
%   finding the first only but also unifying the loaded structure with a
%   single element also ensures the constraint.
%
%   @see https://mavlink.io/en/
%
%   @see https://github.com/mavlink/mavlink/tree/master/message_definitions/v1.0

mavlink_definitions(Base, Mavlink) :-
    file_name_extension(Base, xml, File),
    directory_file_path('/mavlink/mavlink/master/message_definitions/v1.0',
                        File, Path),
    parse_url(URL, [ protocol(https),
                     host('raw.githubusercontent.com'),
                     path(Path)
                   ]),
    load_structure(URL, [Element], []),
    xpath_chk(Element, /(mavlink), Mavlink).

mavlink_definition(Mavlink, Definition) :-
    (   mavlink_enum_definition(Mavlink, Definition)
    ;   mavlink_message_definition(Mavlink, Definition)
    ).

mavlink_enum_definition(Mavlink, Definition) :-
    mavlink_enum(Mavlink, EnumName, Options, Enum),
    (   mavlink_enum(EnumName, Options) = Definition
    ;   mavlink_enum_entry_definition(Enum, EnumName, Definition)
    ).

mavlink_enum_entry_definition(Enum, EnumName,
                              mavlink_enum_entry(EnumName,
                                                 EntryName,
                                                 Value,
                                                 Options)) :-
    mavlink_enum_entry(Enum, EntryName, Value, Options).

mavlink_message_definition(Mavlink, Definition) :-
    mavlink_message(Mavlink, MessageName, Id, Options, Message),
    (   mavlink_message(MessageName, Id, Options) = Definition
    ;   mavlink_message_field_definition(Message, MessageName, Definition)
    ).

mavlink_message_field_definition(Message, MessageName,
                                 mavlink_message_field(MessageName,
                                                       FieldName,
                                                       Type,
                                                       Options)) :-
    mavlink_message_field(Message, FieldName, Type, Options).

%!  mavlink_enum(+Mavlink, ?EnumName, -Options, -Enum) is nondet.

mavlink_enum(Mavlink, EnumName, Options, Enum) :-
    xpath(Mavlink, enums/enum(@name=EnumName), Enum),
    attrs_options(Enum, [name=_], Options).

%!  mavlink_enum_entry(+Enum, ?EntryName, ?Value, -Options) is nondet.

mavlink_enum_entry(Enum, EntryName, Value, Options) :-
    xpath(Enum, entry(@value(number)=Value,
                      @name=EntryName), Entry),
    attrs_options(Entry, [value=_, name=_], Options).

mavlink_message(Mavlink, MessageName, Id, Options, Message) :-
    xpath(Mavlink, messages/message(@id(number)=Id,
                                    @name=MessageName), Message),
    attrs_options(Message, [id=_, name=_], Options).

mavlink_message_field(Message, FieldName, Type, Options) :-
    xpath(Message, field(@type=Type,
                         @name=FieldName), Field),
    attrs_options(Field, [type=_, name=_], Options).

attrs_options(element(_, Attrs, _), Delete, Options) :-
    subtract(Attrs, Delete, Attrs_),
    merge_options(Attrs_, [], Options).
