var eyebrowsShape = 'angular';
var eyebrowsRotation = 0;
var eyebrowsHeight = 0.3;
var eyelidsHeight = 0.1;
// var blinkingInterval = 4000;
var eyeballsDirection = 0.2;
var eyeballsIntensity = 0.4;
var mouthEmotion = 'neutral';

var json;


function emotionValues() {
  console.log([
    'eyebrowsShape: "{0}",',
    ' eyebrowsRotation: {1},',
    ' eyebrowsHeight: {2},',
    ' eyelidsHeight: {3},',
    ' eyeballsDirection: {4},',
    ' eyeballsIntensity: {5},',
    ' mouthEmotion: "{6}"'].join('\n').format(
    eyebrowsShape,      // 0
    eyebrowsRotation,   // 1
    eyebrowsHeight,     // 2
    eyelidsHeight,      // 3
    eyeballsDirection,  // 4
    eyeballsIntensity,  // 5
    mouthEmotion        // 6
  ));
}

function updateJson() {
  json = ['{',
          '  "eyebrows": {',
          '    "shapes": {',
          '      "both_sides": "{0}"',
          '    },',
          '    "transform": {',
          '      "both_sides": {',
          '        "rotation": {1},',
          '        "height": {2}',
          '      }',
          '    }',
          '  },',
          '  "eyelids": {',
          '    "heights": {',
          '      "both_sides": {3}',
          '    }',
          // '    },',
          // '    "blinking_interval": {4}',
          '  },',
          '  "eyeballs": {',
          '    "positions": {',
          '      "both_sides": {',
          '        "direction": {4},',
          '        "intensity": {5}',
          '      }',
          '    }',
          '  },',
          '  "mouth": {',
          '    "emotion": "{6}"',
          '  }',
          '}'].join('\n').format(
            eyebrowsShape,      // 0
            eyebrowsRotation,   // 1
            eyebrowsHeight,     // 2
            eyelidsHeight,      // 3
            // blinkingInterval,   // 4
            eyeballsDirection,  // 5
            eyeballsIntensity,  // 6
            mouthEmotion        // 7
          )
}

function changeValues() {
  eyebrowsShape = $('input[name=eyebrowsShape]:checked').val();
  eyebrowsRotation = parseFloat($('#slider-eyebrowsRotation').val());
  eyebrowsHeight = parseFloat($('#slider-eyebrowsHeight').val());
  eyelidsHeight = parseFloat($('#slider-eyelidsHeight').val());
  // blinkingInterval = parseFloat($('#slider-blinkingInterval').val());
  eyeballsDirection = parseFloat($('#slider-eyeballsDirection').val());
  eyeballsIntensity = parseFloat($('#slider-eyeballsIntensity').val());
  mouthEmotion = $('input[name=mouthEmotion]:checked').val();
  updateJson();
  // console.log(json);
  RobotIcon.parseAndApplyJson(json);
}

String.prototype.format = function() {
    var formatted = this;
    for (var i = 0; i < arguments.length; i++) {
        var regexp = new RegExp('\\{'+i+'\\}', 'gi');
        formatted = formatted.replace(regexp, arguments[i]);
    }
    return formatted;
};


function addControls() {
  $('body').append( '<div id="manual-controls" style="color:#cccccc;width:100%;">' +
                    '  <div id="radio-eyebrowsShape" class="control-group">' +
                    '    <label for="radio-eyebrowsShape-angular">' +
                    '      <input class="control" type="radio" name="eyebrowsShape" id="radio-eyebrowsShape-angular" value="angular" checked="true">' +
                    '      angular' +
                    '    </label><br/>' +
                    '    <label for="radio-eyebrowsShape-round">' +
                    '      <input class="control" type="radio" name="eyebrowsShape" id="radio-eyebrowsShape-round" value="round">' +
                    '      round' +
                    '    </label><br/>' +
                    '  </div>' +
                    '  <div class="control-group">' +
                    '    <label for="slider-eyebrowsRotation">' +
                    '      <p>eyebrowsRotation</p>' +
                    '      <input class="control" id="slider-eyebrowsRotation" type="range" min="-30" max="40" step="1", value="0">' +
                    '    </label>' +
                    '  </div>' +
                    '  <div class="control-group">' +
                    '    <label for="slider-eyebrowsHeight">' +
                    '      <p>eyebrowsHeight</p>' +
                    '      <input class="control" id="slider-eyebrowsHeight" type="range" min="0" max="1" step="0.05", value="0.35">' +
                    '    </label>' +
                    '  </div>' +
                    '  <div class="control-group">' +
                    '    <label for="slider-eyelidsHeight">' +
                    '      <p>eyelidsHeight</p>' +
                    '      <input class="control" id="slider-eyelidsHeight" type="range" min="0" max="1" step="0.05", value="0.0">' +
                    '    </label>' +
                    '  </div>' +
                    '  <div class="control-group">' +
                    '    <label for="slider-eyeballsDirection">' +
                    '      <p>eyeballsDirection</p>' +
                    '      <input class="control" id="slider-eyeballsDirection" type="range" min="0" max="360" step="1", value="0">' +
                    '    </label>' +
                    '  </div>' +
                    '  <div class="control-group">' +
                    '    <label for="slider-eyeballsIntensity">' +
                    '      <p>eyeballsIntensity</p>' +
                    '      <input class="control" id="slider-eyeballsIntensity" type="range" min="0" max="1" step="0.05", value="0">' +
                    '    </label>' +
                    '  </div>' +
                    '  <div id="radio-mouthEmotion" class="control-group">' +
                    '    <label for="radio-mouthEmotion-neutral">' +
                    '      <input class="control" type="radio" name="mouthEmotion" id="radio-mouthEmotion-neutral" value="neutral"  checked="true">' +
                    '      neutral' +
                    '    </label><br/>' +
                    '    <label for="radio-mouthEmotion-happy">' +
                    '      <input class="control" type="radio" name="mouthEmotion" id="radio-mouthEmotion-happy" value="happy">' +
                    '      happy' +
                    '    </label><br/>' +
                    '    <label for="radio-mouthEmotion-angry">' +
                    '      <input class="control" type="radio" name="mouthEmotion" id="radio-mouthEmotion-angry" value="angry">' +
                    '      angry' +
                    '    </label><br/>' +
                    '    <label for="radio-mouthEmotion-sad">' +
                    '      <input class="control" type="radio" name="mouthEmotion" id="radio-mouthEmotion-sad" value="sad">' +
                    '      sad' +
                    '    </label><br/>' +
                    '    <label for="radio-mouthEmotion-uncertain">' +
                    '      <input class="control" type="radio" name="mouthEmotion" id="radio-mouthEmotion-uncertain" value="uncertain">' +
                    '      uncertain' +
                    '    </label><br/>' +
                    '  </div>' +
                    '</div>');
  $('.control-group').css('float','left');

  $('.control').change(changeValues);
}


$(document).ready(function() {
  addControls();
  setTimeout(function(){$('.face').height($(document).height());}, 100);
});
