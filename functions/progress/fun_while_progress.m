function fun_while_progress(a0,a,a_conv)

N=100;

if a0==0 && a==0 && a_conv==0
    fun_for_progress(N,1);
    return;
end

log_a0=log(a0);

log_a=log(a);

log_a1=log(a_conv);


if a>=a0
    i=1.001;
elseif a<=a_conv
    i=N;
else
    i=(log_a0-log_a)/(log_a0-log_a1)*N;
end


fun_for_progress(N,i);