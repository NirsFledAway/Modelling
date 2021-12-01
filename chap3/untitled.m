f()
f()
function f()
    persistent val
    if isempty(val)
        val = [struct('v', [1 2 3]), struct('v', [4 5 5])]
    end
    val
end