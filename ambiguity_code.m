function [ambig] = ambiguity_code(input,fd)
N = size(input,2);
tao = N;                            %��ʱ����
code = input;                       %����
samp = size(code,2)*10;             %ȡ���źŸ���
n = ceil(log(samp)/log(2));         
nfft = 2^n;                         %fft����
u(1:nfft) = 0;                      
j = 0;
for index = 1:10:samp
    j = j+1;
    u(index:index+10-1) = code(j);  %�����ź�
end
v = u;
delay = linspace(0,5*tao,nfft);     %��ʱ
freq_del = 2*fd/tao/100;              %������Ƶ��Ƶ��
j = 0;
vfft = fft(v,nfft);                 %�����ź�fft
for freq = -fd/tao:freq_del:fd/tao
    j = j+1;
    exf = exp(1i*2*pi*freq.*delay); 
    u_times = u.*exf;               %Ƶ�ơ�ʱ�Ӻ��ź�
    ufft = fft(u_times,nfft);       %Ƶ�ơ�ʱ�Ӻ��ź�fft
    prod = ufft.*conj(vfft);        %Ƶ�����
    ambig(j,:) = fftshift(abs(ifft(prod)));%ʱ���������֣�
end