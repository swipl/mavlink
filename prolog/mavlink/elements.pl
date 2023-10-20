/*  File:    mavlink/elements.pl
    Author:  Roy Ratcliffe
    Created: Aug 13 2023
    Purpose: MAVLink Elements
*/

:- module(mavlink_elements,
          []).
:- autoload(library(apply), [maplist/3]).
:- autoload(library(lists), [reverse/2]).
:- autoload(library(option),
            [select_option/4, select_option/3, option/2, option/3]).
:- autoload(library(dcg/basics), [csym/3, integer/3]).
:- use_module(library(debug), [debugging/1, debug/3]).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

content([], Options, Options).
content([H|T], Options0, Options) :-
    element(H, Options0, Options_),
    content(T, Options_, Options).

element(element(Tag, Attrs, Content), Options0, [tags(Tags)|Options]) :-
    !,
    element(Tag, Attrs, Content, Options0, Options_),
    select_option(tags(Tags), Options_, Options__, []),
    content(Content, [tags([Tag|Tags])|Options__], Options___),
    select_option(tags(_), Options___, Options).
element(_, Options, Options).

element(Tag, Attrs, _Content, Options, Options) :-
    option(tags(Tags), Options, []),
    atomic_list_concat([Tag|Tags], '_', Name),
    Term =.. [Name|Attrs],
    writeq(Term),
    nl.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

mavlink_terms(Mavlink, Terms) :-
    phrase(element([], Options), [Mavlink]),
    memberchk(terms(Terms0), Options),
    reverse(Terms0, Terms).

content(Options, Options) --> [].
content(Options0, Options) -->
    element(Options0, Options_),
    content(Options_, Options).

element(Options0, [elements(Elements)|Options]) -->
    [element(Tag, Attrs, Content)],
    !,
    { element_(Tag, Attrs, Options0, Options_),
      select_option(elements(Elements), Options_, Options__, []),
      phrase(content([ elements([ element(Tag, Attrs, Content)
                                | Elements
                                ])
                     | Options__
                     ], Options___), Content),
      !,
      select_option(elements(_), Options___, Options)
    }.
element(Options, Options) --> [_].

element_(enum, Attrs, Options0, Options) :-
    args(1, [enums, mavlink], Options0),
    !,
    select_option(name(Name), Attrs, Attrs1),
    term(enum(Name, Attrs1), Options0, Options).
element_(message, Attrs, Options0, [extensions(false)|Options]) :-
    !,
    select_option(name(Name), Attrs, Attrs1),
    select_option(id(Id), Attrs1, Attrs2),
    atom_number(Id, Id1),
    term(message(Name, Id1, Attrs2), Options0, Options_),
    select_option(extensions(_), Options_, Options, _).
element_(extensions, _, Options0, [extensions(true)|Options]) :-
    !,
    select_option(extensions(false), Options0, Options).
element_(field, Attrs, Options0, Options) :-
    args(1, [message, messages, mavlink], Options0),
    !,
    args(2, [MessageAttrs|_], Options0),
    select_option(name(Name), Attrs, Attrs1),
    select_option(type(Type), Attrs1, Attrs2),
    atom_codes(Type, Codes),
    phrase(type(Type1), Codes),
    option(name(MessageName), MessageAttrs),
    option(extensions(Extensions), Options0),
    term(message_field(MessageName, Name, Type1,
                       [ extensions(Extensions)
                       | Attrs2
                       ]), Options0, Options).
element_(Tag, Attrs, Options, Options) :-
    (   debugging(mavlink(elements))
    ->  args(1, Tags, Options),
        atomic_list_concat([Tag|Tags], '_', Name),
        Term =.. [Name|Attrs],
        debug(mavlink(elements), '~q', [Term])
    ;   true
    ).

term(H, Options0, Options) :-
    select_option_(H, terms(_), Options0, Options).

select_option_(H, Option, Options0, [Option_|Options]) :-
    select_option(Option, Options0, Options, []),
    Option =.. [Name, T],
    Option_ =.. [Name, [H|T]].

args(Arg, Args, Options) :-
    option(elements(Elements), Options, []),
    maplist(arg(Arg), Elements, Args).

type(Array) -->
    csym(Basic),
    "[",
    !,
    integer(Len),
    { between(1, 255, Len)
    },
    "]",
    { Array =.. [Basic, Len]
    }.
type(Basic) --> csym(Basic).

description(enum, enums, enum_description).
description(message, messages, message_description).

description(entry, enum, enums, enum_entry_description).
description(field, message, messages, message_field_description).

deprecated(enum, enums, enum_deprecated).
deprecated(message, messages, message_deprecated).

deprecated(entry, enum, enums, enum_entry_deprecated).
deprecated(field, message, messages, message_field_deprecated).
