sinearray = zeros(128);
sawtootharray = zeros(128);
trianglearray = zeros(128);

t = 0:128;

sineLUT = dsp.SineWave('Amplitude',1,'Frequency',1.25,'SampleRate',128);
sineLUT.SamplesPerFrame = 128;

subplot(3,1,1);

x = sawtooth(0.2*t,1/2);
plot(t,x)
xlim([0 128])

sawtoothAnalog = sawtooth(0.2*t);

z = sawtoothAnalog;
y = sineLUT();

subplot(3,1,2)
plot(y)
xlim([0 128])

subplot(3,1,3)
plot(t, z)
xlim([0 128])

size(x);
size(y);
size(z);
x_temp = x(1,:);
y_temp = y(:,1);
z_temp = z(1,:);
size(x_temp);
for t = 1:128
    sinearray(t) = x_temp(1,t);
    sawtootharray(t) = y_temp(t,1);
    trianglearray(t) = z_temp(1,t);
end
sinearray(:,1);
sawtootharray(:,1);
trianglearray(:,1)