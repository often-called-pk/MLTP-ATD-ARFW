function v = getfielddef(s, name, default)
%GETFIELDDEF Struct field with fallback. v = s.(name) if present, else default.
if isstruct(s) && isscalar(s) && isfield(s, name)
    v = s.(name);
else
    v = default;
end
end
