function [JL_inv] = Jacobian_inv(l1,l2,q)

s1 = sin(q(1));
c1 = cos(q(1));
s2 = sin(q(2));
c2 = cos(q(2));
c12 = cos(q(1)+q(2));
s12 = sin(q(1)+q(2));

Jl11 = c12/(l1*s2);
Jl12 = s12/(l1*s2);
Jl21 = -(l2*c12 + l1*c1)/(l1*l2*s2);
Jl22 = -(l2*s12 + l1*s1)/(l1*l2*s2);

JL_inv = [Jl11 Jl12;
          Jl21 Jl22];
end

