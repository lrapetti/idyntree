function v = LINK_MOMENT_OF_INERTIA_XX()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 39);
  end
  v = vInitialized;
end
