var RobotIcon = (function () {
  var snap;
  var face;
  var templates;
  var blinkingInterval = 4000;
  var blinkingTimeout;
  var blinkingReopenTimeout;

  function ensureWithinRange(variable, min, max) {
    // default values
    if (min === undefined) min = 0;
    if (max === undefined) max = 1;
    return Math.min(Math.max(variable, min), max);
  }

  function mergeIntoLeftRight(values) {
    var bothSides = values.both_sides;
    if (typeof bothSides !== 'undefined') {
      if (typeof bothSides === 'object') {
        return {
          left:  jQuery.extend(true, {}, bothSides, values.left),
          right: jQuery.extend(true, {}, bothSides, values.right)
        };
      }
      return {
        left: typeof values.left !== 'undefined' ? values.left : bothSides,
        right: typeof values.right !== 'undefined' ? values.right : bothSides
      }
    }
    return values;
  }

  function animateEyeballsDirection(positions, duration) {
    positions = mergeIntoLeftRight(positions);
    if (duration == undefined) duration = 100;

    Object.keys(positions).forEach(function(side) {
      var intensity = ensureWithinRange(positions[side].intensity);
      // The eyes can move within a circle of 18
      var x = Math.sin(positions[side].direction * Math.PI / 180) * 18 * intensity;
      var y = Math.cos(positions[side].direction * Math.PI / 180) * 18 * intensity;

      face.eyeballs[side].group.obj.stop().animate({
        transform: (new Snap.Matrix()).translate(x, y)
      }, duration, mina.easein);
      face.eyeballs[side].group.val.direction = positions[side].direction;
      face.eyeballs[side].group.val.intensity = intensity;
    })
  }

  function animateEyeballsColor(color, duration) {
    console.warn('Color changing for eyeballs is not yet implemented.');
  }

  function animateEyebrowsShape(new_shapes, duration) {
    var new_shapes = mergeIntoLeftRight(new_shapes);
    if (duration == undefined) duration = 100;

    Object.keys(new_shapes).forEach(function(side) {
      face.eyebrows[side].obj.eyebrow.animate({
        d: templates.eyebrows[side][new_shapes[side]].attr('d')
      }, duration, mina.easeinout);
      face.eyebrows[side].val.shape = new_shapes[side];
    })
  }

  function animateEyebrowsTransform(transform, duration) {
    animateEyebrowsRotation(transform, duration);
    animateEyebrowsHeight(transform, duration);
  }

  function animateEyebrowsHeight(transform, duration) {
    transform = mergeIntoLeftRight(transform);
    if (duration == undefined) duration = 100;

    Object.keys(transform).forEach(function(side) {
      var height = transform[side].height;
      if (typeof height !== 'undefined') {
        var transformMatrix = new Snap.Matrix();
        // -19 is the lowest good looking position for the eyebrows,
        // +10 is the highest one, therefore norm height to:
        height = - (ensureWithinRange(height) * 29 - 10);
        transformMatrix.translate(0, height);
        face.eyebrows[side].val.height = height;

        face.eyebrows[side].obj.box.animate({
          transform: transformMatrix
        }, duration, mina.easeout);
        face.eyebrows[side].val.transform = jQuery.extend(true, {}, face.eyebrows[side].val.transform, transform[side]);
      }
    })
  }

  function animateEyebrowsRotation(transform, duration) {
    transform = mergeIntoLeftRight(transform);
    if (duration == undefined) duration = 100;

    Object.keys(transform).forEach(function(side) {
      var rotation = transform[side].rotation;
      if (typeof rotation !== 'undefined') {
        var transformMatrix = new Snap.Matrix();
        // (35,15) is the optimal center of the eyebrow rotation for both sides
        transformMatrix.rotate(side === 'right' ? -rotation : rotation, 35, 15);
        face.eyebrows[side].val.rotation = rotation;

        face.eyebrows[side].obj.eyebrow.animate({
          transform: transformMatrix
        }, duration, mina.easeout);
        face.eyebrows[side].val.transform = jQuery.extend(true, {}, face.eyebrows[side].val.transform, transform[side]);
      }
    })
  }

  function animateEyebrowsColor(color, duration) {
    console.warn('Color changing for eyebrows is not yet implemented.');
  }

  function animateEyelids(heights, duration, isBlinking) {
    if (! isBlinking) clearBlinkingTimeouts();
    heights = mergeIntoLeftRight(heights);
    if (duration == undefined) duration = 100;

    Object.keys(heights).forEach(function(side) {
      var translation = new Snap.Matrix();
      translation.translate(0, ensureWithinRange(heights[side]) * 52);
      face.eyelids[side].obj.animate({
        transform: translation
      }, duration, mina.easeout);
      if (! isBlinking) face.eyelids[side].val.height = heights[side];
    })

    if (! isBlinking) updateBlinkingInterval();
  }

  function animateMouth(emotion, duration) {
    if (duration == undefined) duration = 100;
    face.mouth.obj.stop().animate({
      d: templates.mouth[emotion].attr('d')
    }, duration, mina.easeinout);
    face.mouth.val.emotion = emotion;
  }

  function animateHairColor(color, duration) {
    console.warn('Color changing for hair is not yet implemented.');
  }

  function animateSkinColor(color, duration) {
    console.warn('Color changing for skin is not yet implemented.');
  }

  function clearBlinkingTimeouts() {
    clearTimeout(blinkingTimeout);
  }

  function blinkEyes() {
    var closingDuration = 50;
    var closedDuration = 100;
    var openingDuration = 80;

    animateEyelids({left: 1, right: 1}, closingDuration, true);
    setTimeout(function(){
        var normalClosedness = {
          left: face.eyelids.left.val.height,
          right: face.eyelids.right.val.height
        }
        animateEyelids(normalClosedness, openingDuration, true);
      }, closingDuration + closedDuration);
  }

  function blinkEyesInIntervals() {
    blinkEyes();
    updateBlinkingInterval()
  }

  function updateBlinkingInterval(newBlinkingInterval) {
    if (newBlinkingInterval)
      blinkingInterval = newBlinkingInterval;
    clearBlinkingTimeouts();
    blinkingTimeout = setTimeout(
        blinkEyesInIntervals,
        newBlinkingInterval ? blinkingInterval / 2 : blinkingInterval
      );
  }

  function parseAndApplyJson(json, duration) {
    var input = JSON.parse(json);
    duration = duration || 100;

    if (input.eyebrows) {
      var newShapes, transform, newColor;
      if (newShapes = input.eyebrows.shapes)
        animateEyebrowsShape(newShapes, duration);
      if (transform = input.eyebrows.transform)
        animateEyebrowsTransform(transform, duration);
      if (newColor = input.eyebrows.color)
        animateEyebrowsColor(newColor, duration);
    }

    if (input.eyelids) {
      var newHeight, newBlinkingInterval;
      if (newHeight = input.eyelids.heights)
        animateEyelids(newHeight, duration);
      if (newBlinkingInterval = input.eyelids.blinking_interval)
        updateBlinkingInterval(newBlinkingInterval);
    }

    if (input.eyeballs) {
      var newPositions, newColors;
      if (newPositions = input.eyeballs.positions)
        animateEyeballsDirection(newPositions, duration);
      if (newColors = input.eyeballs.colors)
        animateEyeballsColor(newColors, duration);
    }

    if (input.mouth) {
      var emotion;
      if (emotion = input.mouth.emotion)
        animateMouth(emotion, duration);
    }

    if (input.hair) {
      var newColor;
      if (newColor = input.hair.color)
        animateHairColor(newColor, duration);
    }

    if (input.skin) {
      var newColor;
      if (newColor = input.skin.color)
        animateSkinColor(newColor, duration);
    }
  }

  function setFaces() {
    face = {
      eyebrows: {
        left: {
          obj: {
            eyebrow: snap.select('#eyebrow-left-animated'),
            box: snap.select('#eyebrow-left-box'),
            rotationCenter: snap.select('#eyebrow-left-center')
          },
          val: {shape: 'angular', rotation: 0, height: 0.35}
        },
        right: {
          obj: {
            eyebrow: snap.select('#eyebrow-right-animated'),
            box: snap.select('#eyebrow-right-box'),
            rotationCenter: snap.select('#eyebrow-right-center')
          },
          val: {shape: 'angular', rotation: 0, height: 0.35}
        }
      },
      eyelids: {
        left: {
          obj: snap.select('#eye-left-lid'),
          val: {height: 0}
        },
        right: {
          obj: snap.select('#eye-right-lid'),
          val: {height: 0}
        }
      },
      eyeballs: {
        left: {
          iris: {obj: snap.select('#eye-left-iris')},
          pupil: {obj: snap.select('#eye-left-pupil')},
          group: {
            obj: snap.select('#eyeball-left'),
            val: {direction: 0, intensity: 0}
          }
        },
        right: {
          iris: {obj: snap.select('#eye-right-iris')},
          pupil: {obj: snap.select('#eye-right-pupil')},
          group: {
            obj: snap.select('#eyeball-right'),
            val: {direction: 0, intensity: 0}
          }
        }
      },
      mouth: {
        obj: snap.select('#mouth-animated'),
        val: {emotion: 'neutral'}
      }
    }

    templates = {eyebrows: {left: {}, right: {}}, mouth: {}};
    ['neutral', 'happy', 'angry', 'sad', 'uncertain'].forEach(function(emotion) {
      templates.mouth[emotion] = snap.select('#mouth-' + emotion)
    });
    ['angular', 'round'].forEach(function(shape) {
        templates.eyebrows.left[shape] = snap.select('#eyebrow-left-' + shape);
        templates.eyebrows.right[shape] = snap.select('#eyebrow-right-' + shape);
    });
  }

  function displayWholeFace() {
    snap.attr({
      viewBox: '0 0 291 336'
    })
  }

  function displayEyesOnly() {
    snap.attr({
      viewBox: '73 110 146 168'
    })
  }

  function displayEyesMouth() {
    snap.attr({
      //original viewBox: '73 140 146 178'
	  // desplazamiento en x, desplaz. y, 
//	  viewBox: '135 140 120 250'
	  viewBox: '145 150 120 250'
    })
  }

  function changeDisplayMode(mode) {
    switch (mode) {
      case 'whole_face':
        displayWholeFace();
        break;
      case 'eyes_only':
        displayEyesOnly();
        break;
      case 'eyes_mouth':
        displayEyesMouth();
        break;
    }
  }

  function getSnapObj() { return snap; }

  function getFace() {
    return face;
  }

  function initialize() {
    var faceContainer = $('.roboticon');
    faceContainer.height(faceContainer.parent().height());
    snap = Snap('.roboticon');
    snap.attr({
      height: faceContainer.parent().height(),
      // Widescreen format: 16 x 9
      width:  Math.floor(faceContainer.parent().height() / 9 * 16),
    });
    setFaces();
    updateBlinkingInterval(blinkingInterval);
  }

  return {
    initialize: initialize,
    parseAndApplyJson: parseAndApplyJson,
    blinkEyes: blinkEyes,
    animateEyeballsDirection: animateEyeballsDirection,
    animateEyebrowsShape: animateEyebrowsShape,
    animateEyebrowsTransform: animateEyebrowsTransform,
    // animateEyebrowsHeight: animateEyebrowsHeight,
    // animateEyebrowsRotation: animateEyebrowsRotation,
    animateEyelids: animateEyelids,
    animateMouth: animateMouth,
    displayWholeFace: displayWholeFace,
    displayEyesOnly: displayEyesOnly,
    changeDisplayMode: changeDisplayMode,
    updateBlinkingInterval: updateBlinkingInterval,
    __getInternalFace__: getFace
  };
})();

$('.roboticon').load(function() {
  RobotIcon.initialize();
})
