// Static geometry (walls, floor, ceiling, fixtures).
export const PRIMITIVES = [
  {
    "id": "apartment-floor", 
    "type": "box", 
    "name": "Apartment Floor Slab", 
    "dimensions": {
      "width": 12, 
      "height": 0.1, 
      "depth": 10
    }, 
    "transform": {
      "position": {
        "x": 0, 
        "y": 0.00813020206586168, 
        "z": 0
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#D2B48C", 
      "softness": 0.4, 
      "metalness": 0.1, 
      "roughness": 0.4, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "front-yard-floor", 
    "type": "box", 
    "name": "Front Yard Grass", 
    "dimensions": {
      "width": 12, 
      "height": 0.05, 
      "depth": 4
    }, 
    "transform": {
      "position": {
        "x": 0, 
        "y": 0.025, 
        "z": 7
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#4F7942", 
      "softness": 0.92, 
      "metalness": 0, 
      "roughness": 0.92, 
      "hardness": 0.12, 
      "fluffiness": 0.72, 
      "specularIntensity": 0.25, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 0.18, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.4, 
      "thickness": 0.04, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0.95, 
      "textureSoftness": 0.7, 
      "textureHardness": 0.18, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 4.4, 
        "repeatY": 1.1, 
        "offsetX": 0.23, 
        "offsetY": 0, 
        "rotationDeg": -7
      }, 
      "texturePath": "textures/dc34e383b17a.jpg"
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "yard-wall-south", 
    "type": "box", 
    "name": "Yard Wall South", 
    "dimensions": {
      "width": 12.4, 
      "height": 0.5, 
      "depth": 0.2
    }, 
    "transform": {
      "position": {
        "x": 0, 
        "y": 0.25, 
        "z": 9.1
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#808080", 
      "softness": 0.7, 
      "metalness": 0, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "yard-wall-west", 
    "type": "box", 
    "name": "Yard Wall West", 
    "dimensions": {
      "width": 0.2, 
      "height": 0.5, 
      "depth": 4.2
    }, 
    "transform": {
      "position": {
        "x": -6.1, 
        "y": 0.25, 
        "z": 7
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#808080", 
      "softness": 0.7, 
      "metalness": 0, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "yard-wall-east", 
    "type": "box", 
    "name": "Yard Wall East", 
    "dimensions": {
      "width": 0.2, 
      "height": 0.5, 
      "depth": 4.2
    }, 
    "transform": {
      "position": {
        "x": 6.1, 
        "y": 0.25, 
        "z": 7
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#808080", 
      "softness": 0.7, 
      "metalness": 0, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-north", 
    "type": "box", 
    "name": "North Exterior Wall", 
    "dimensions": {
      "width": 12, 
      "height": 3, 
      "depth": 0.2
    }, 
    "transform": {
      "position": {
        "x": 0, 
        "y": 1.6, 
        "z": -5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#F8F8F8", 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-east", 
    "type": "box", 
    "name": "East Exterior Wall", 
    "dimensions": {
      "width": 0.2, 
      "height": 3, 
      "depth": 10
    }, 
    "transform": {
      "position": {
        "x": 6, 
        "y": 1.6, 
        "z": 0
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#0d0d0d", 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-west", 
    "type": "box", 
    "name": "West Exterior Wall", 
    "dimensions": {
      "width": 0.2, 
      "height": 3, 
      "depth": 10
    }, 
    "transform": {
      "position": {
        "x": -6, 
        "y": 1.6, 
        "z": 0
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#5e77b0", 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-south-seg-1", 
    "type": "box", 
    "name": "South Wall Segment Left", 
    "dimensions": {
      "width": 1.5, 
      "height": 3, 
      "depth": 0.2
    }, 
    "transform": {
      "position": {
        "x": -5.25, 
        "y": 1.6, 
        "z": 5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#ffffff", 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-south-seg-2", 
    "type": "box", 
    "name": "South Wall Segment Middle", 
    "dimensions": {
      "width": 4.5, 
      "height": 3, 
      "depth": 0.2
    }, 
    "transform": {
      "position": {
        "x": -1.25, 
        "y": 1.6, 
        "z": 5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#48460e", 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-south-seg-3", 
    "type": "box", 
    "name": "South Wall Segment Right", 
    "dimensions": {
      "width": 3, 
      "height": 3, 
      "depth": 0.2
    }, 
    "transform": {
      "position": {
        "x": 4.5, 
        "y": 1.6, 
        "z": 5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#48460e", 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-south-header-entrance", 
    "type": "box", 
    "name": "South Wall Header Entrance", 
    "dimensions": {
      "width": 1, 
      "height": 0.9, 
      "depth": 0.2
    }, 
    "transform": {
      "position": {
        "x": -4, 
        "y": 2.7, 
        "z": 5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#F8F8F8", 
      "softness": 0.7, 
      "metalness": 0, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-south-header-sliding", 
    "type": "box", 
    "name": "South Wall Header Sliding Door", 
    "dimensions": {
      "width": 2, 
      "height": 0.9, 
      "depth": 0.2
    }, 
    "transform": {
      "position": {
        "x": 2, 
        "y": 2.7, 
        "z": 5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#48460e", 
      "softness": 0.7, 
      "metalness": 0, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-main-left", 
    "type": "box", 
    "name": "Main Divider Left", 
    "dimensions": {
      "width": 5.5, 
      "height": 3, 
      "depth": 0.15
    }, 
    "transform": {
      "position": {
        "x": -3.25, 
        "y": 1.6, 
        "z": 0.012688827212309017
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#F8F8F8", 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-main-right", 
    "type": "box", 
    "name": "Main Divider Right", 
    "dimensions": {
      "width": 5.5, 
      "height": 3, 
      "depth": 0.15
    }, 
    "transform": {
      "position": {
        "x": 3.25, 
        "y": 1.6, 
        "z": 0
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#cacac4", 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-main-header", 
    "type": "box", 
    "name": "Main Divider Header", 
    "dimensions": {
      "width": 1, 
      "height": 0.9, 
      "depth": 0.15
    }, 
    "transform": {
      "position": {
        "x": 0, 
        "y": 2.55, 
        "z": 0
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#F8F8F8", 
      "softness": 0.7, 
      "metalness": 0, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-kitchen-back", 
    "type": "box", 
    "name": "Kitchen Divider Back", 
    "dimensions": {
      "width": 0.15, 
      "height": 3, 
      "depth": 2
    }, 
    "transform": {
      "position": {
        "x": -2, 
        "y": 1.6, 
        "z": 1
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#F8F8F8", 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-kitchen-front", 
    "type": "box", 
    "name": "Kitchen Divider Front", 
    "dimensions": {
      "width": 0.15, 
      "height": 3, 
      "depth": 2
    }, 
    "transform": {
      "position": {
        "x": -2, 
        "y": 1.6, 
        "z": 4
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#F8F8F8", 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-kitchen-header", 
    "type": "box", 
    "name": "Kitchen Divider Header", 
    "dimensions": {
      "width": 0.15, 
      "height": 0.9, 
      "depth": 1
    }, 
    "transform": {
      "position": {
        "x": -2, 
        "y": 2.55, 
        "z": 2.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#F8F8F8", 
      "softness": 0.7, 
      "metalness": 0, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-bathroom-back", 
    "type": "box", 
    "name": "Bathroom Divider Back", 
    "dimensions": {
      "width": 0.15, 
      "height": 3, 
      "depth": 2
    }, 
    "transform": {
      "position": {
        "x": 1, 
        "y": 1.6, 
        "z": -4
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#F8F8F8", 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-bathroom-front", 
    "type": "box", 
    "name": "Bathroom Divider Front", 
    "dimensions": {
      "width": 0.15, 
      "height": 3, 
      "depth": 2
    }, 
    "transform": {
      "position": {
        "x": 1, 
        "y": 1.5434592146356212, 
        "z": -1
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#F8F8F8", 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "wall-bathroom-header", 
    "type": "box", 
    "name": "Bathroom Divider Header", 
    "dimensions": {
      "width": 0.15, 
      "height": 0.9, 
      "depth": 1
    }, 
    "transform": {
      "position": {
        "x": 1, 
        "y": 2.55, 
        "z": -2.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#F8F8F8", 
      "softness": 0.7, 
      "metalness": 0, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "ceiling-slab", 
    "type": "box", 
    "name": "Ceiling Slab", 
    "dimensions": {
      "width": 12, 
      "height": 0.2, 
      "depth": 10
    }, 
    "transform": {
      "position": {
        "x": 0, 
        "y": 3.1, 
        "z": 0
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#FFFFFF", 
      "softness": 1, 
      "roughness": 1, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "downlight-cutout-1", 
    "type": "cylinder", 
    "name": "Downlight Cutout LR", 
    "dimensions": {
      "radiusTop": 0.075, 
      "radiusBottom": 0.075, 
      "height": 0.02
    }, 
    "transform": {
      "position": {
        "x": 3, 
        "y": 3, 
        "z": 2.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#111111", 
      "softness": 0.5, 
      "metalness": 0.2, 
      "roughness": 0.5, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "downlight-cutout-2", 
    "type": "cylinder", 
    "name": "Downlight Cutout Kitchen", 
    "dimensions": {
      "radiusTop": 0.075, 
      "radiusBottom": 0.075, 
      "height": 0.02
    }, 
    "transform": {
      "position": {
        "x": -3, 
        "y": 3, 
        "z": 2.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#111111", 
      "softness": 0.5, 
      "metalness": 0.2, 
      "roughness": 0.5, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "downlight-cutout-3", 
    "type": "cylinder", 
    "name": "Downlight Cutout Bedroom", 
    "dimensions": {
      "radiusTop": 0.075, 
      "radiusBottom": 0.075, 
      "height": 0.02
    }, 
    "transform": {
      "position": {
        "x": 3, 
        "y": 3, 
        "z": -2.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#111111", 
      "softness": 0.5, 
      "metalness": 0.2, 
      "roughness": 0.5, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "downlight-cutout-4", 
    "type": "cylinder", 
    "name": "Downlight Cutout Bathroom", 
    "dimensions": {
      "radiusTop": 0.075, 
      "radiusBottom": 0.075, 
      "height": 0.02
    }, 
    "transform": {
      "position": {
        "x": -3, 
        "y": 3, 
        "z": -2.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#111111", 
      "softness": 0.5, 
      "metalness": 0.2, 
      "roughness": 0.5, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "window-south-frame-top", 
    "type": "box", 
    "name": "Window Frame Top", 
    "dimensions": {
      "width": 2, 
      "height": 0.05, 
      "depth": 0.1
    }, 
    "transform": {
      "position": {
        "x": 2, 
        "y": 2.225, 
        "z": 5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#2C2C2C", 
      "softness": 0.3, 
      "metalness": 0.7, 
      "roughness": 0.3, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "window-south-frame-bottom", 
    "type": "box", 
    "name": "Window Frame Bottom", 
    "dimensions": {
      "width": 2, 
      "height": 0.05, 
      "depth": 0.1
    }, 
    "transform": {
      "position": {
        "x": 2, 
        "y": 0.11827884470220063, 
        "z": 5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#2C2C2C", 
      "softness": 0.3, 
      "metalness": 0.7, 
      "roughness": 0.3, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "window-south-frame-left", 
    "type": "box", 
    "name": "Window Frame Left", 
    "dimensions": {
      "width": 0.05, 
      "height": 1.4, 
      "depth": 0.1
    }, 
    "transform": {
      "position": {
        "x": 1.025, 
        "y": 1.5, 
        "z": 5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 2.026631513257506, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#2C2C2C", 
      "softness": 0.3, 
      "metalness": 0.7, 
      "roughness": 0.3, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "window-south-frame-right", 
    "type": "box", 
    "name": "Window Frame Right", 
    "dimensions": {
      "width": 0.05, 
      "height": 1.4, 
      "depth": 0.1
    }, 
    "transform": {
      "position": {
        "x": 2.975, 
        "y": 1.5, 
        "z": 5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 2.057602018219857, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#2C2C2C", 
      "softness": 0.3, 
      "metalness": 0.7, 
      "roughness": 0.3, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "window-south-glass", 
    "type": "box", 
    "name": "Window Glass", 
    "dimensions": {
      "width": 1.9, 
      "height": 1.4, 
      "depth": 0.02
    }, 
    "transform": {
      "position": {
        "x": 2, 
        "y": 1.498572723371466, 
        "z": 5.021528258120427
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 2.061706354585481, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#ADD8E6", 
      "opacity": 0.3, 
      "transmission": 0.9, 
      "ior": 1.5, 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "window-east-frame-top", 
    "type": "box", 
    "name": "Window Frame Top", 
    "dimensions": {
      "width": 0.1, 
      "height": 0.05, 
      "depth": 2
    }, 
    "transform": {
      "position": {
        "x": 6, 
        "y": 2.225, 
        "z": -2
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#2C2C2C", 
      "softness": 0.3, 
      "metalness": 0.7, 
      "roughness": 0.3, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "window-east-frame-bottom", 
    "type": "box", 
    "name": "Window Frame Bottom", 
    "dimensions": {
      "width": 0.1, 
      "height": 0.05, 
      "depth": 2
    }, 
    "transform": {
      "position": {
        "x": 6, 
        "y": 0.775, 
        "z": -2
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#2C2C2C", 
      "softness": 0.3, 
      "metalness": 0.7, 
      "roughness": 0.3, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "window-east-frame-left", 
    "type": "box", 
    "name": "Window Frame Left", 
    "dimensions": {
      "width": 0.1, 
      "height": 1.4, 
      "depth": 0.05
    }, 
    "transform": {
      "position": {
        "x": 6, 
        "y": 1.5, 
        "z": -2.975
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#2C2C2C", 
      "softness": 0.3, 
      "metalness": 0.7, 
      "roughness": 0.3, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "window-east-frame-right", 
    "type": "box", 
    "name": "Window Frame Right", 
    "dimensions": {
      "width": 0.1, 
      "height": 1.4, 
      "depth": 0.05
    }, 
    "transform": {
      "position": {
        "x": 6, 
        "y": 1.5, 
        "z": -1.025
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#2C2C2C", 
      "softness": 0.3, 
      "metalness": 0.7, 
      "roughness": 0.3, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "window-east-glass", 
    "type": "box", 
    "name": "Window Glass", 
    "dimensions": {
      "width": 0.02, 
      "height": 1.4, 
      "depth": 1.9
    }, 
    "transform": {
      "position": {
        "x": 6, 
        "y": 1.5, 
        "z": -2
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#ADD8E6", 
      "opacity": 0.3, 
      "transmission": 0.9, 
      "ior": 1.5, 
      "softness": 0.7, 
      "roughness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "lr-light-trim-1", 
    "type": "cylinder", 
    "name": "LR Light Trim 1", 
    "dimensions": {
      "radiusTop": 0.08, 
      "radiusBottom": 0.08, 
      "height": 0.02
    }, 
    "transform": {
      "position": {
        "x": 0, 
        "y": 3, 
        "z": 1.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#111111", 
      "softness": 0.5, 
      "metalness": 0.2, 
      "roughness": 0.5, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "castShadow": false, 
    "receiveShadow": false, 
    "physics": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "lr-light-trim-2", 
    "type": "cylinder", 
    "name": "LR Light Trim 2", 
    "dimensions": {
      "radiusTop": 0.08, 
      "radiusBottom": 0.08, 
      "height": 0.02
    }, 
    "transform": {
      "position": {
        "x": 4, 
        "y": 3, 
        "z": 1.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#111111", 
      "softness": 0.5, 
      "metalness": 0.2, 
      "roughness": 0.5, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "castShadow": false, 
    "receiveShadow": false, 
    "physics": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "lr-light-trim-4", 
    "type": "cylinder", 
    "name": "LR Light Trim 4", 
    "dimensions": {
      "radiusTop": 0.08, 
      "radiusBottom": 0.08, 
      "height": 0.02
    }, 
    "transform": {
      "position": {
        "x": 4, 
        "y": 3, 
        "z": 3.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#111111", 
      "softness": 0.5, 
      "metalness": 0.2, 
      "roughness": 0.5, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "castShadow": false, 
    "receiveShadow": false, 
    "physics": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "kitchen-light-trim-1", 
    "type": "cylinder", 
    "name": "Kitchen Light Trim 1", 
    "dimensions": {
      "radiusTop": 0.08, 
      "radiusBottom": 0.08, 
      "height": 0.02
    }, 
    "transform": {
      "position": {
        "x": -4, 
        "y": 3, 
        "z": 1.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#111111", 
      "softness": 0.5, 
      "metalness": 0.2, 
      "roughness": 0.5, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "castShadow": false, 
    "receiveShadow": false, 
    "physics": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "kitchen-light-trim-2", 
    "type": "cylinder", 
    "name": "Kitchen Light Trim 2", 
    "dimensions": {
      "radiusTop": 0.08, 
      "radiusBottom": 0.08, 
      "height": 0.02
    }, 
    "transform": {
      "position": {
        "x": -4, 
        "y": 3, 
        "z": 3.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#111111", 
      "softness": 0.5, 
      "metalness": 0.2, 
      "roughness": 0.5, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "castShadow": false, 
    "receiveShadow": false, 
    "physics": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "bedroom-light-trim-1", 
    "type": "cylinder", 
    "name": "Bedroom Light Trim 1", 
    "dimensions": {
      "radiusTop": 0.08, 
      "radiusBottom": 0.08, 
      "height": 0.02
    }, 
    "transform": {
      "position": {
        "x": -2.5, 
        "y": 3, 
        "z": -1.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#111111", 
      "softness": 0.5, 
      "metalness": 0.2, 
      "roughness": 0.5, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "castShadow": false, 
    "receiveShadow": false, 
    "physics": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "bedroom-light-trim-2", 
    "type": "cylinder", 
    "name": "Bedroom Light Trim 2", 
    "dimensions": {
      "radiusTop": 0.08, 
      "radiusBottom": 0.08, 
      "height": 0.02
    }, 
    "transform": {
      "position": {
        "x": -2.5, 
        "y": 3, 
        "z": -3.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#111111", 
      "softness": 0.5, 
      "metalness": 0.2, 
      "roughness": 0.5, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "castShadow": false, 
    "receiveShadow": false, 
    "physics": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "bathroom-light-trim-1", 
    "type": "cylinder", 
    "name": "Bathroom Light Trim 1", 
    "dimensions": {
      "radiusTop": 0.08, 
      "radiusBottom": 0.08, 
      "height": 0.02
    }, 
    "transform": {
      "position": {
        "x": 3.5, 
        "y": 3, 
        "z": -2.5
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#111111", 
      "softness": 0.5, 
      "metalness": 0.2, 
      "roughness": 0.5, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "castShadow": false, 
    "receiveShadow": false, 
    "physics": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "path-light-base-1", 
    "type": "cylinder", 
    "name": "Path Light Base 1", 
    "dimensions": {
      "radiusTop": 0.03, 
      "radiusBottom": 0.03, 
      "height": 0.2
    }, 
    "transform": {
      "position": {
        "x": -2, 
        "y": 0.1, 
        "z": 5.8
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#333333", 
      "softness": 0.1, 
      "metalness": 0.9, 
      "roughness": 0.1, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "path-light-base-2", 
    "type": "cylinder", 
    "name": "Path Light Base 2", 
    "dimensions": {
      "radiusTop": 0.03, 
      "radiusBottom": 0.03, 
      "height": 0.2
    }, 
    "transform": {
      "position": {
        "x": 0, 
        "y": 0.1, 
        "z": 5.8
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#333333", 
      "softness": 0.1, 
      "metalness": 0.9, 
      "roughness": 0.1, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "path-light-base-3", 
    "type": "cylinder", 
    "name": "Path Light Base 3", 
    "dimensions": {
      "radiusTop": 0.03, 
      "radiusBottom": 0.03, 
      "height": 0.2
    }, 
    "transform": {
      "position": {
        "x": 2, 
        "y": 0.1, 
        "z": 5.8
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1, 
        "y": 1, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#333333", 
      "softness": 0.1, 
      "metalness": 0.9, 
      "roughness": 0.1, 
      "hardness": 0, 
      "fluffiness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "textureDataUrl": null, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "alphaCutoff": 0, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true, 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}
  }, 
  {
    "id": "783be8417c9b68-19c6daa8d28", 
    "type": "plane", 
    "name": "Wooden Floor", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": 1.9499811797054045, 
        "y": 0.11479516914077526, 
        "z": 2.5224651049954567
      }, 
      "rotation": {
        "x": 1.5707963267948966, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 3.973075253030828, 
        "y": 2.505288504802451, 
        "z": 19.684670163409347
      }
    }, 
    "material": {
      "color": "#808080", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "texturePath": "textures/faada43680d3.jpg"
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "8335d17d79d7d-19c72918c44", 
    "type": "plane", 
    "name": "Plane copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -2.548736547541645, 
        "y": 0.09589921676402557, 
        "z": -2.462454631636581
      }, 
      "rotation": {
        "x": 1.5707963267948966, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 3.5707846155541514, 
        "y": 2.505288504802451, 
        "z": 19.684670163409347
      }
    }, 
    "material": {
      "color": "#808080", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "texturePath": "textures/faada43680d3.jpg"
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "04f41436813cd-19c7293d8e6", 
    "type": "plane", 
    "name": "Plane", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": 3.3425399746150166, 
        "y": 0.08145220992406443, 
        "z": -2.270485912724944
      }, 
      "rotation": {
        "x": 1.5707963267948966, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 2.7523616637617936, 
        "y": 2.7523616637617936, 
        "z": 2.7523616637617936
      }
    }, 
    "material": {
      "color": "#808080", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 5.1, 
        "repeatY": 5.3, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "texturePath": "textures/08e5e2b877e1.jpg"
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "c087837b444638-19c7296d766", 
    "type": "plane", 
    "name": "Plane", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -3.7303788186939344, 
        "y": 0.08969239741599833, 
        "z": 2.5666047855931198
      }, 
      "rotation": {
        "x": 1.5707963267948966, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1.7457717771446934, 
        "y": 2.5261863196647547, 
        "z": 2.8705851724882394
      }
    }, 
    "material": {
      "color": "#808080", 
      "roughness": 0.96, 
      "softness": 0.96, 
      "hardness": 0.88, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 0.2, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 0.15, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.5, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 1, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.35, 
      "textureHardness": 0.7, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 3.5, 
        "repeatY": 5.9, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "texturePath": "textures/af07500a616c.webp"
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "0379edf13da0a-19c72992dab", 
    "type": "box", 
    "name": "Box", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 1, 
      "height": 1, 
      "depth": 1, 
      "edgeRadius": 0, 
      "edgeSegments": 4, 
      "widthSegments": 1, 
      "heightSegments": 1, 
      "depthSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -3.99963558334846, 
        "y": 0.05741370387386052, 
        "z": 5.032630876866396
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1.0307684092832463, 
        "y": 0.07524662771800945, 
        "z": 0.1327083917447404
      }
    }, 
    "material": {
      "color": "#747272", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true
  }, 
  {
    "id": "ea4192c9414a48-19c730f2895", 
    "type": "plane", 
    "name": "Plane", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": 5.87425878966027, 
        "y": 1.5871495527977475, 
        "z": 2.4898086797049364
      }, 
      "rotation": {
        "x": 0, 
        "y": 1.5707963267948966, 
        "z": 0
      }, 
      "scale": {
        "x": 2.4340540365918795, 
        "y": 1.4798606990825292, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#48460e", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "f943a66bf590a8-19c737557be", 
    "type": "box", 
    "name": "Box", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 1, 
      "height": 1, 
      "depth": 1, 
      "edgeRadius": 0, 
      "edgeSegments": 4, 
      "widthSegments": 1, 
      "heightSegments": 1, 
      "depthSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -5.281145415825732, 
        "y": 0.571374473094583, 
        "z": 2.7080407264295236
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.02118022478582253, 
        "y": 0.7608388096193667, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#5c5c5c", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true
  }, 
  {
    "id": "7b36be69ab5f9-19c73779a39", 
    "type": "box", 
    "name": "counter top", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 1, 
      "height": 1, 
      "depth": 1, 
      "edgeRadius": 0, 
      "edgeSegments": 4, 
      "widthSegments": 1, 
      "heightSegments": 1, 
      "depthSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -5.592825880628662, 
        "y": 0.9629473017917998, 
        "z": 3.979374310570936
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.6415432111938124, 
        "y": 0.030534569005993762, 
        "z": 1.8728951285475166
      }
    }, 
    "material": {
      "color": "#222222", 
      "roughness": 0.9, 
      "softness": 0.9, 
      "hardness": 0.58, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 0.4, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 0.25, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.52, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0.85, 
      "clearcoatRoughness": 0.19999999999999996, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.55, 
      "textureHardness": 0.25, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true
  }, 
  {
    "id": "1edfcf9d820c38-19c737b3dea", 
    "type": "box", 
    "name": "Box copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 1, 
      "height": 1, 
      "depth": 1, 
      "edgeRadius": 0, 
      "edgeSegments": 4, 
      "widthSegments": 1, 
      "heightSegments": 1, 
      "depthSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -5.601804712793383, 
        "y": 0.1597592283515767, 
        "z": 2.70050519498599
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.6104781025633932, 
        "y": 0.06677847501178394, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#222222", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true
  }, 
  {
    "id": "300c8b5b9d2268-19c737cddea", 
    "type": "box", 
    "name": "Box copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 1, 
      "height": 1, 
      "depth": 1, 
      "edgeRadius": 0, 
      "edgeSegments": 4, 
      "widthSegments": 1, 
      "heightSegments": 1, 
      "depthSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -5.599990138028935, 
        "y": 0.5709156037037538, 
        "z": 0.7005983724176564
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.6104781025633932, 
        "y": 0.7765067159413703, 
        "z": 1.3006021507278882
      }
    }, 
    "material": {
      "color": "#5c5c5c", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true
  }, 
  {
    "id": "57e0f28d450758-19c737da478", 
    "type": "box", 
    "name": "Box copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 1, 
      "height": 1, 
      "depth": 1, 
      "edgeRadius": 0, 
      "edgeSegments": 4, 
      "widthSegments": 1, 
      "heightSegments": 1, 
      "depthSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -5.602253941815052, 
        "y": 0.1597592283515767, 
        "z": 1.6632802784622847
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.6104781025633932, 
        "y": 0.06677847501178394, 
        "z": 2.9846531143176267
      }
    }, 
    "material": {
      "color": "#222222", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true
  }, 
  {
    "id": "dc03685ba8afc-19c737eac11", 
    "type": "box", 
    "name": "Box copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 1, 
      "height": 1, 
      "depth": 1, 
      "edgeRadius": 0, 
      "edgeSegments": 4, 
      "widthSegments": 1, 
      "heightSegments": 1, 
      "depthSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -5.28004311005808, 
        "y": 0.5709156037037538, 
        "z": 2.9376932413135677
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.031910310947402244, 
        "y": 0.7320533043664592, 
        "z": 0.42713125248605477
      }
    }, 
    "material": {
      "color": "#5c5c5c", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true
  }, 
  {
    "id": "a70fbe76145d98-19c737f8136", 
    "type": "box", 
    "name": "Box copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 1, 
      "height": 1, 
      "depth": 1, 
      "edgeRadius": 0, 
      "edgeSegments": 4, 
      "widthSegments": 1, 
      "heightSegments": 1, 
      "depthSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -5.275241230716765, 
        "y": 0.5709156037037538, 
        "z": 2.4463799300425264
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.031910310947402244, 
        "y": 0.7320533043664592, 
        "z": 0.42713125248605477
      }
    }, 
    "material": {
      "color": "#5c5c5c", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true
  }, 
  {
    "id": "d5ac39c69ea1f-19c738008ac", 
    "type": "box", 
    "name": "Box", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 1, 
      "height": 1, 
      "depth": 1, 
      "edgeRadius": 0, 
      "edgeSegments": 4, 
      "widthSegments": 1, 
      "heightSegments": 1, 
      "depthSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -5.257358385471168, 
        "y": 0.5912214945385093, 
        "z": 2.7706532825233054
      }, 
      "rotation": {
        "x": 1.5707963267948966, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.01, 
        "y": 0.01, 
        "z": 0.52
      }
    }, 
    "material": {
      "color": "#808080", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true
  }, 
  {
    "id": "0b93f0188a3a98-19c738106dc", 
    "type": "box", 
    "name": "Box copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 1, 
      "height": 1, 
      "depth": 1, 
      "edgeRadius": 0, 
      "edgeSegments": 4, 
      "widthSegments": 1, 
      "heightSegments": 1, 
      "depthSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -5.258383417198247, 
        "y": 0.5868688079397046, 
        "z": 2.5937656709903236
      }, 
      "rotation": {
        "x": 1.5707963267948966, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.01, 
        "y": 0.01, 
        "z": 0.52
      }
    }, 
    "material": {
      "color": "#808080", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true
  }, 
  {
    "id": "10258e2601f01-19c73b500a9", 
    "type": "box", 
    "name": "counter top copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 1, 
      "height": 1, 
      "depth": 1, 
      "edgeRadius": 0, 
      "edgeSegments": 4, 
      "widthSegments": 1, 
      "heightSegments": 1, 
      "depthSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -5.590946496428486, 
        "y": 0.9629473017917998, 
        "z": 1.1978787402098119
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.6415432111938124, 
        "y": 0.030534569005993762, 
        "z": 2.2517642186060898
      }
    }, 
    "material": {
      "color": "#222222", 
      "roughness": 0.9, 
      "softness": 0.9, 
      "hardness": 0.58, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 0.4, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 0.25, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.52, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0.85, 
      "clearcoatRoughness": 0.19999999999999996, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.55, 
      "textureHardness": 0.25, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": true, 
    "receiveShadow": true
  }, 
  {
    "id": "757f5b99740bc8-19c7421407d", 
    "type": "plane", 
    "name": "Plane", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -5.896691185276851, 
        "y": 1.9002281170120718, 
        "z": 2.497801452904678
      }, 
      "rotation": {
        "x": 0, 
        "y": 1.5707963267948966, 
        "z": 0
      }, 
      "scale": {
        "x": 2.4067091733612673, 
        "y": 1.105765175939889, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#0c122c", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "6529c964ed9c48-19c742206f5", 
    "type": "plane", 
    "name": "Plane copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -3.9945150774550386, 
        "y": 1.548538143174976, 
        "z": 0.09109105470202561
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1.9150984716058357, 
        "y": 1.6501678483888989, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#0c122c", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "3cdf3c74e21738-19c7423fa90", 
    "type": "plane", 
    "name": "Plane copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -2.0866166516533178, 
        "y": 1.55, 
        "z": 0.9879211662233722
      }, 
      "rotation": {
        "x": 0, 
        "y": 1.5707963267948966, 
        "z": 0
      }, 
      "scale": {
        "x": 1.0098416828841905, 
        "y": 1.65, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#0c122c", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "e211a7cfba6098-19c742616ce", 
    "type": "plane", 
    "name": "Plane copy copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -2.079468429661268, 
        "y": 1.55, 
        "z": 4.008907836090804
      }, 
      "rotation": {
        "x": 0, 
        "y": 1.5707963267948966, 
        "z": 0
      }, 
      "scale": {
        "x": 1.0098416828841905, 
        "y": 1.65, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#0c122c", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "3832ba4a663b18-19c74266e2c", 
    "type": "plane", 
    "name": "Plane copy copy copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -2.0905822043840847, 
        "y": 2.667278164723253, 
        "z": 2.7832702590785026
      }, 
      "rotation": {
        "x": 0, 
        "y": 1.5707963267948966, 
        "z": 0
      }, 
      "scale": {
        "x": 1.0098416828841905, 
        "y": 0.5635515294365225, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#0c122c", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "7f1aa5007dc09-19c74275609", 
    "type": "plane", 
    "name": "Plane copy copy copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -2.732367750607517, 
        "y": 1.55, 
        "z": 4.897304524794526
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.7687618378437209, 
        "y": 1.65, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#0c122c", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "15c49fe32fa1c-19c74288cdb", 
    "type": "plane", 
    "name": "Plane copy copy copy copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -5.268666684592633, 
        "y": 1.55, 
        "z": 4.897304524794526
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.7687618378437209, 
        "y": 1.65, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#0c122c", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "2a7cdd6f9d04f-19c74290c86", 
    "type": "plane", 
    "name": "Plane copy copy copy copy copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -1.266158206687444, 
        "y": 1.633247104283046, 
        "z": 5.100329993926027
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 2.2343213628098764, 
        "y": 1.545936740677122, 
        "z": 1.556053916200829
      }
    }, 
    "material": {
      "color": "#ffffff", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "1e6229faca67f8-19c742b9800", 
    "type": "plane", 
    "name": "Plane copy copy copy copy copy copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": 4.519250048552278, 
        "y": 1.633247104283046, 
        "z": 5.103878449570321
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1.5092307395484803, 
        "y": 1.545936740677122, 
        "z": 1.556053916200829
      }
    }, 
    "material": {
      "color": "#ffffff", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "1b661a4c44dd48-19c742c52f8", 
    "type": "plane", 
    "name": "Plane copy copy copy copy copy copy copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": 2.178431996727124, 
        "y": 2.723672602796335, 
        "z": 5.103878449570321
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 1.5092307395484803, 
        "y": 0.44396423456135964, 
        "z": 1.556053916200829
      }
    }, 
    "material": {
      "color": "#ffffff", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "634e0ced21dc88-19c83a32fb2", 
    "type": "plane", 
    "name": "Plane copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": 3.1865602637106494, 
        "y": 1.59, 
        "z": 0.08072901728527726
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 2.685299750697142, 
        "y": 1.48, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#48460e", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "dbc3c36393b35-19c83a4558e", 
    "type": "plane", 
    "name": "Plane copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -1.2201811715191955, 
        "y": 1.593493435339756, 
        "z": 0.09116940361175385
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.7227000672566991, 
        "y": 1.48, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#48460e", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "ec87b8c8dfa178-19c83a50983", 
    "type": "plane", 
    "name": "Plane copy copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -0.16284675486099442, 
        "y": 2.672862262421784, 
        "z": 0.09116940361175385
      }, 
      "rotation": {
        "x": 0, 
        "y": 0, 
        "z": 0
      }, 
      "scale": {
        "x": 0.7227000672566991, 
        "y": 0.567054231994336, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#48460e", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "5bb11b4c6365a8-19c83a61793", 
    "type": "plane", 
    "name": "Plane copy copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -1.91726728103272, 
        "y": 1.59, 
        "z": 1.0092475796780658
      }, 
      "rotation": {
        "x": 0, 
        "y": 1.5707963267948966, 
        "z": 0
      }, 
      "scale": {
        "x": 0.9938495526728357, 
        "y": 1.48, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#48460e", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "bbac7ab73352e8-19c83a727f7", 
    "type": "plane", 
    "name": "Plane copy copy copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -1.922284295625794, 
        "y": 1.59, 
        "z": 3.9969772404633046
      }, 
      "rotation": {
        "x": 0, 
        "y": 1.5707963267948966, 
        "z": 0
      }, 
      "scale": {
        "x": 0.9938495526728357, 
        "y": 1.48, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#48460e", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }, 
  {
    "id": "fc2c976a67628-19c83a7bab9", 
    "type": "plane", 
    "name": "Plane copy copy copy copy", 
    "notes": "", 
    "tags": [], 
    "state": "static", 
    "metadata": {}, 
    "dimensions": {
      "width": 2, 
      "height": 2, 
      "widthSegments": 1, 
      "heightSegments": 1
    }, 
    "transform": {
      "position": {
        "x": -1.9085894055370365, 
        "y": 2.627961007913602, 
        "z": 2.678720608499149
      }, 
      "rotation": {
        "x": 0, 
        "y": 1.5707963267948966, 
        "z": 0
      }, 
      "scale": {
        "x": 0.9938495526728357, 
        "y": 0.5209561440322126, 
        "z": 1
      }
    }, 
    "material": {
      "color": "#48460e", 
      "roughness": 0.7, 
      "softness": 0.7, 
      "hardness": 0, 
      "fluffiness": 0, 
      "metalness": 0, 
      "specularIntensity": 1, 
      "specularColor": "#ffffff", 
      "envMapIntensity": 1, 
      "opacity": 1, 
      "transmission": 0, 
      "ior": 1.45, 
      "thickness": 0, 
      "attenuationColor": "#ffffff", 
      "attenuationDistance": 1, 
      "iridescence": 0, 
      "emissive": "#000000", 
      "emissiveIntensity": 0, 
      "clearcoat": 0, 
      "clearcoatRoughness": 0, 
      "alphaCutoff": 0, 
      "textureSoftness": 0.25, 
      "textureHardness": 0.5, 
      "doubleSided": true, 
      "flatShading": false, 
      "wireframe": false, 
      "uvTransform": {
        "repeatX": 1, 
        "repeatY": 1, 
        "offsetX": 0, 
        "offsetY": 0, 
        "rotationDeg": 0
      }, 
      "textureDataUrl": null
    }, 
    "physics": true, 
    "castShadow": false, 
    "receiveShadow": true
  }
];
