out = timerfind;

for i = 1:length(out)
    stop(out(i));
    out(i).delete;
end