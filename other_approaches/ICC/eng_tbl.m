% This is the M-file to calculate engine torque for given engine speed 
% and throttle angle.

%
% Jianlong Zhang
% Department of Electrical Engineering
% University of Southern California
% June 09, 2005
%

function eng_out=eng_tbl(u)

we=u(1);
in_value=u(2);

engine_table;             % put the engine table in variable 'tabledata'

in_index=5;             % input: throttle angle
out_index=2;            % output: engine torque

ind_I=1; ind_J=1; ind_K=1; 

if(we < tabledata(1,1))
    ind_I = 1;
    we = tabledata(1,1);
elseif(we >= tabledata(255,1))
    ind_I = 16;
    we = tabledata(255,1);
else
    for i=1:1:16
        if( (we >= tabledata(i*15-14,1) & we < tabledata(i*15+1,1) ) |(we <= tabledata(i*15-14,1) & we > tabledata(i*15+1,1) ) )
            ind_I=i;
            break;
        end
    end
end
             
if( in_value < tabledata(ind_I*15-14,in_index) )
    ind_J = 1;      % in_value = tabledata(ind_I*15-14,in_index);
elseif( in_value >= tabledata(ind_I*15,in_index) )
    ind_J = 14;     % in_value = tabledata(ind_I*15,in_index); 
else
    for j = 1:1:14
        if( ((in_value >= tabledata((ind_I-1)*15 + j,in_index)) & (in_value < tabledata((ind_I-1)*15+ j + 1,in_index)))|((in_value <= tabledata((ind_I-1)*15 + j,in_index)) & (in_value > tabledata((ind_I-1)*15+ j + 1,in_index))))
            ind_J=j; break;
        end
    end
end

if( in_value < tabledata(ind_I*15+1,in_index) )
    ind_K = 1;      % in_value = tabledata((ind_I*15+1,in_index);
elseif( in_value >= tabledata((ind_I + 1)*15,in_index) )
    ind_K = 14;     % in_value = tabledata((ind_I + 1)*15,in_index);
else
    for k = 1:1:14
        if( (in_value >= tabledata(ind_I*15 + k,in_index) & in_value < tabledata(ind_I*15 +k +1,in_index) ) |(in_value <= tabledata(ind_I*15 + k,in_index)& in_value > tabledata(ind_I*15 +k +1,in_index) ) )
            ind_K=k; break;
        end
    end
end

a = we - tabledata(ind_I*15-14,1);                                   
b = tabledata(ind_I*15+1,1) - we;

ta = (tabledata(ind_I*15+ ind_K,in_index) * a+ tabledata((ind_I-1)*15+ind_J,in_index) * b) / (a + b);
outa = (tabledata(ind_I*15+ind_K,out_index) * a+ tabledata((ind_I-1)*15+ind_J,out_index) * b) / (a + b);

tb = (tabledata(ind_I *15  + ind_K + 1,in_index) * a+ tabledata(ind_I*15-14+ind_J,in_index) * b) / (a + b);
outb = (tabledata(ind_I *15  + ind_K + 1,out_index) * a+ tabledata(ind_I*15-14+ind_J,out_index) * b) / (a + b);

a = in_value - ta;
b = tb - in_value;

if (a==0)&(b==0)
    r=outa;
    disp('warning 1 in the function eng_tbl');
elseif ((a+b)==0)
    r=outa;
    disp('warning 2 in the function eng_tbl');
else
    r = (outb*a + outa*b)/(a + b);
end
eng_out=r;