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
          [ mavlink_assert_definitions_r/2,     % +Base,+M
            mavlink_definitions_r/2,            % +Base,-Mavlinks:list
            mavlink_definitions/2               % +Base,-Mavlink
          ]).
:- autoload(library(filesex), [directory_file_path/3]).
:- autoload(library(lists), [reverse/2, subtract/3, append/3, member/2]).
:- autoload(library(pairs), [pairs_keys/2]).
:- autoload(library(sgml), [load_structure/3]).
:- autoload(library(url), [parse_url/2]).
:- autoload(library(xpath), [xpath/3, xpath_chk/3]).
:- autoload(library(option), [select_option/3, option/2]).
:- autoload(library(apply), [maplist/3]).
:- use_module(library(xpath)).

%!  mavlink_assert_definitions_r(+Base, +M) is semidet.

mavlink_assert_definitions_r(Base, M) :-
    mavlink_definitions_r(Base, Mavlinks),
    forall(member(Base_-Mavlink, Mavlinks),
           element(Mavlink, M, [element(Base_, _, _)])).

element(element(Tag, Attrs, Content), M, Contain) :-
    !,
    element(M:element(Tag, Attrs, Content), Contain),
    content(Content, M, [element(Tag, Attrs, Content)|Contain]).
element(_, _M, _Contain).

content([], _M, _Contain).
content([H|T], M, Contain) :-
    element(H, M, Contain),
    content(T, M, Contain).

element(M:element(enum, EnumAttrs, _),
        [ element(enums, _, _),
          element(mavlink, _, _),
          element(Base, _, _)
        ]) :-
    !,
    select_option(name(EnumName), EnumAttrs, EnumAttrs1),
    assertz(M:enum(EnumName, EnumAttrs1)),
    assertz(M:enum_base(EnumName, Base)).
element(M:element(entry, EntryAttrs, _),
        [ element(enum, EnumAttrs, _),
          element(enums, _, _),
          element(mavlink, _, _),
          element(_Base, _, _)
        ]) :-
    !,
    select_option(name(EntryName), EntryAttrs, EntryAttrs1),
    select_option(value(Value), EntryAttrs1, EntryAttrs2),
    option(name(EnumName), EnumAttrs),
    atom_number(Value, Value1),
    assertz(M:enum_entry(EnumName, EntryName, Value1, EntryAttrs2)).
element(M:element(param, ParamAttrs, _),
        [ element(entry, EntryAttrs, _),
          element(enum, EnumAttrs, _),
          element(enums, _, _),
          element(mavlink, _, _),
          element(_Base, _, _)
        ]) :-
    !,
    select_option(index(Index), ParamAttrs, ParamAttrs1),
    option(name(EntryName), EntryAttrs),
    option(name(EnumName), EnumAttrs),
    atom_number(Index, Index1),
    assertz(M:enum_entry_param(EnumName, EntryName, Index1, ParamAttrs1)).
element(M:element(message, MessageAttrs, _),
        [ element(messages, _, _),
          element(mavlink, _, _),
          element(Base, _, _)
        ]) :-
    !,
    select_option(name(MessageName), MessageAttrs, MessageAttrs1),
    select_option(id(Id), MessageAttrs1, MessageAttrs2),
    atom_number(Id, Id1),
    assertz(M:message(MessageName, Id1, MessageAttrs2)),
    assertz(M:message_base(MessageName, Base)),
    ignore(retract(M:message_extensions(MessageName, _))).
element(M:element(field, FieldAttrs, _),
        [ element(message, MessageAttrs, _),
          element(messages, _, _),
          element(mavlink, _, _),
          element(_Base, _, _)
        ]) :-
    !,
    select_option(name(FieldName), FieldAttrs, FieldAttrs1),
    select_option(type(Type), FieldAttrs1, FieldAttrs2),
    option(name(MessageName), MessageAttrs),
    assertz(M:message_field(MessageName, FieldName, Type, FieldAttrs2)),
    (   retract(M:message_extensions(MessageName, T))
    ->  assertz(M:message_extensions(MessageName, [FieldName|T]))
    ;   true
    ).
element(M:element(extensions, _, _),
        [ element(message, MessageAttrs, _),
          element(messages, _, _),
          element(mavlink, _, _),
          element(_Base, _, _)
        ]) :-
    !,
    option(name(MessageName), MessageAttrs),
    assertz(M:message_extensions(MessageName, [])).
element(M:element(description, _, [Description]),
        [ element(SubTag, SubAttrs, _),
          element(SuperTag, _, _),
          element(mavlink, _, _),
          element(_Base, _, _)
        ]) :-
    description(SubTag, SuperTag, Name),
    !,
    option(name(SubName), SubAttrs),
    Term =.. [Name, SubName, Description],
    assertz(M:Term).
element(M:element(description, _, [Description]),
        [ element(SubSubTag, SubSubAttrs, _),
          element(SubTag, SubAttrs, _),
          element(SuperTag, _, _),
          element(mavlink, _, _),
          element(_Base, _, _)
        ]) :-
    description(SubSubTag, SubTag, SuperTag, Name),
    !,
    option(name(SubSubName), SubSubAttrs),
    option(name(SubName), SubAttrs),
    Term =.. [Name, SubName, SubSubName, Description],
    assertz(M:Term).
element(M:element(wip, _, _),
        [ element(message, MessageAttrs, _),
          element(messages, _, _),
          element(mavlink, _, _),
          element(_Base, _, _)
        ]) :-
    !,
    option(name(MessageName), MessageAttrs),
    assertz(M:message_wip(MessageName)).
element(M:element(deprecated, DeprecatedAttrs, _),
        [ element(SubTag, SubAttrs, _),
          element(SuperTag, _, _),
          element(mavlink, _, _),
          element(_Base, _, _)
        ]) :-
    deprecated(SubTag, SuperTag, Deprecated),
    !,
    option(name(SubName), SubAttrs),
    Term =.. [Deprecated, SubName, DeprecatedAttrs],
    assertz(M:Term).
element(M:element(deprecated, DeprecatedAttrs, _),
        [ element(SubSubTag, SubSubAttrs, _),
          element(SubTag, SubAttrs, _),
          element(SuperTag, _, _),
          element(mavlink, _, _),
          element(_Base, _, _)
        ]) :-
    deprecated(SubSubTag, SubTag, SuperTag, Deprecated),
    !,
    option(name(SubSubName), SubSubAttrs),
    option(name(SubName), SubAttrs),
    Term =.. [Deprecated, SubName, SubSubName, DeprecatedAttrs],
    assertz(M:Term).
element(M:element(Tag, Attrs, _Content), Contain) :-
    maplist(arg(1), Contain, Tags),
    atomic_list_concat([Tag|Tags], '_', Name),
    Term =.. [Name|Attrs],
    writeq(M:Term),
    nl.

description(enum, enums, enum_description).
description(message, messages, message_description).

description(entry, enum, enums, enum_entry_description).
description(field, message, messages, message_field_description).

deprecated(enum, enums, enum_deprecated).
deprecated(message, messages, message_deprecated).

deprecated(entry, enum, enums, enum_entry_deprecated).
deprecated(field, message, messages, message_field_deprecated).

%!  mavlink_definitions_r(+Base, -Mavlinks:list) is det.
%
%   Recursively downloads multiple XML elements containing MAVLink
%   enumeration and message definitions.
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
%       `mavlink` enumeration and message definitions.

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
%   Downloads MAVLink enumeration and message definitions from GitHub.
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
    (   enum_definition(Mavlink, Definition)
    ;   message_definition(Mavlink, Definition)
    ).

enum_definition(Mavlink, Definition) :-
    enum(Mavlink, EnumName, Options, Enum),
    (   enum(EnumName, Options) = Definition
    ;   enum_entry_definition(Enum, EnumName, Definition)
    ).

enum_entry_definition(Enum, EnumName,
                      enum_entry(EnumName,
                                 EntryName,
                                 Value,
                                 Options)) :-
    enum_entry(Enum, EntryName, Value, Options).

message_definition(Mavlink, Definition) :-
    message(Mavlink, MessageName, Id, Options, Message),
    (   message(MessageName, Id, Options) = Definition
    ;   message_field_definition(Message, MessageName, Definition)
    ).

message_field_definition(Message, MessageName,
                         message_field(MessageName,
                                       FieldName,
                                       Type,
                                       Options)) :-
    message_field(Message, FieldName, Type, Options).

enum(Mavlink, EnumName, Options, Enum) :-
    xpath(Mavlink, enums/enum(@name=EnumName), Enum),
    attrs_options(Enum, [name=_], Options).

enum_entry(Enum, EntryName, Value, Options) :-
    xpath(Enum, entry(@value(number)=Value,
                      @name=EntryName), Entry),
    attrs_options(Entry, [value=_, name=_], Options).

message(Mavlink, MessageName, Id, Options, Message) :-
    xpath(Mavlink, messages/message(@id(number)=Id,
                                    @name=MessageName), Message),
    attrs_options(Message, [id=_, name=_], Options).

message_field(Message, FieldName, Type, Options) :-
    xpath(Message, field(@type=Type,
                         @name=FieldName), Field),
    attrs_options(Field, [type=_, name=_], Options).

attrs_options(element(_, Attrs, _), Delete, Options) :-
    subtract(Attrs, Delete, Attrs_),
    merge_options(Attrs_, [], Options).

message_element(element(message, _, Elements),
                element(Name, Attrs, Content)) :-
    member(element(Name, Attrs, Content), Elements).

message_elements(Message, Elements) :-
    findall(Element, message_element(Message, Element), Elements).

message_fields(Message, Fields) :-
    message_elements(Message, Elements0),
    message_fields_(Elements0, [], Fields, _).

message_fields_([], Attrs, [], Attrs).
message_fields_([element(field, FieldAttrs0, _)|Elements0], Attrs0,
                [element(field, FieldAttrs, _)|Elements], Attrs) :-
    !,
    append(FieldAttrs0, Attrs0, FieldAttrs),
    message_fields_(Elements0, Attrs0, Elements, Attrs).
message_fields_([element(extensions, _, _)|Elements0], Attrs0,
                Elements, Attrs) :-
    !,
    message_fields_(Elements0, [extensions=true|Attrs0], Attrs, Elements).
message_fields_([_|Elements0], Attrs0,
                Elements, Attrs) :-
    message_fields_(Elements0, Attrs0, Attrs, Elements).
