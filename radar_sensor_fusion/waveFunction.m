function wave = waveFunction(t, slope, fc)
    omega = fc*t + slope*t*t/2;
    wave = cos(2*pi*omega);
end