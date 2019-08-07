function mj_equality(eqid, state)
%mj_equality(eqid, state)
%   change state of equality constraint; state must be 1 or 0

mjhx('equality', eqid, state);

end
