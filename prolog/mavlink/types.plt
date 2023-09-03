:- begin_tests(mavlink_types).
:- use_module(types).

test(mavlink_type_atom_size, [true(A==1)]) :-
    mavlink_type_atom_size(uint8_t, A).
test(mavlink_type_atom_size, [fail]) :-
    mavlink_type_atom_size('uint8_t[256]', _).

:- end_tests(mavlink_types).
