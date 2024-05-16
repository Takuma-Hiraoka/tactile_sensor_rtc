```yaml
tactile_sensor:
  -
    name: tactile_sensor0 # センサの名前. 重複禁止
    link: RLEG_ANKLE_R # 親リンク名. (VRMLのjoint名ではなく、URDFのリンク名)
    translation: [ -0.065, -0.065, -0.06 ] # 親リンク相対
    rotation: [ 1, 0, 0, 0 ] # 親リンク相対. [axis-x, axis-y, axis-z, angle(rad)]
```
