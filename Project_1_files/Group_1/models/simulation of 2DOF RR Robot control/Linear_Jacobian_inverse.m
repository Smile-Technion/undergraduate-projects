function [inv_JL] = Linear_Jacobian_inverse(q,l1, l2)

c1 = cos(q(1));
c2 = cos(q(2));
s1 = sin(q(1));
s2 = sin(q(2));
c12 = cos(q(1)+q(2));
s12 = sin(q(1)+q(2));

jl11 = c12/(l1*s2);
jl12 = s12/(l1*s2);
jl21 = -(l2*c12+l1*c1)/(l1*l2*s2) ;
jl22 = -(l2*s12+l1*s1)/(l1*l2*s2) ;

inv_JL = [ jl11 jl12;
           jl21 jl22 ];

        
end

