function vals = gen_idx(sizes)
    vals = cell(length(sizes), 1);
    prevIdx = 1;
    for i = 1:length(sizes)
        nextIdx = prevIdx + sizes(i);
        vals{i} = (prevIdx : nextIdx - 1);
        prevIdx = nextIdx;
    end
end