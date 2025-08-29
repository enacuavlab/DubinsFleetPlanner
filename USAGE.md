# Dubins Synchronous Path planning program for fleets of fixed-wing aircraft

## Usage

`DubinsFleetPlanner data_file separation [wind_x] [wind_y] {-flags}`

- `data_file` : A CSV file containing the base data for the problem. It's header (first line) must be:
   `ac_id,start_x,start_y,start_z,start_theta,end_x,end_y,end_z,end_theta,airspeed,climb,turn_radius,dt`

   The fields describe respectively, for each aircraft:
   * `ac_id`              : An integer identifying the aircraft
   * `start_[xyz|(theta)]`: The starting position in XYZ space and XY orientation (theta, in radian) in the ground referential. Unit is up to the user and noted [l]
   * `end_[xyz|(theta)]`  : The ending position in XYZ space and XY orientation (theta, in radian) in the ground referential. Unit is up to the user and noted [l]
   * `airspeed`           : Positive, speed of the aircraft in the air referential, in [l]/[T], with [T] a user-defined time unit
   * `climb`              : Positive, climb speed of the aircraft (and used symmetrically for descent).
                             Altitude unit is up to user and noted [alt], such that climb is in [alt]/[T]
   * `turn_radius`        : Positive, radius length of the tightest turn the aircraft is capable of. Unit is [l]
   * `dt`                 : Time difference at arrival with respect to the previous aircraft (irrelevant for the first one). Unit is [T].

   An example of units may be [l]=meters, [alt]=meters, [T]=seconds ; or [l]=nautical miles, [alt]=feet, [T]=minutes.

                  AS OF NOW, THE THIRD AXIS (Z) IS NOT IMPLEMENTED. THE VALUES SHOULD STILL BE FILLED IN, BUT ARE IRRELEVANT.

- `separation` : A positive value defining the minimum (Euclidean) distance that must be kept between two aircraft at all times. Unit is [l].
                 (See flag `-Z` for how to handle different planar and vertical units.)

                 AS OF NOW, THE THIRD AXIS (Z) IS NOT IMPLEMENTED. SEPARATION IS ONLY CONSIDERED AS IF ALL AIRCRAFT ARE ON THE SAME XY PLANE.

- `wind_x,wind_y`: Values describing the wind speed relative to the ground, respectively along the X and Y axes. If not set, default to 0.
                   Unit is [l]/[T].

- `{-flags}`: List of options that may be set:
   * `-Z [modifier]`: If set, enable considering the Z axis for separation. The optional argument `modifier` set the coefficient `a` in the
                      distance computations such that `d = sqrt(Dx² + Dy² + (a Dz)²)`. This can be used to convert from [alt] to [l] and
                      consider differently vertical distance from the horizontal one. If not set, default to 1.

                      AS OF NOW, THE THIRD AXIS (Z) IS NOT IMPLEMENTED. THIS FLAG HAS NO EFFECTS

   * `-c`: Disable collision detection
   * `-v`: Verbose print to stdout
   * `-s samples`: Defines a number of samples. Used for plotting and saving in sampled form. If the flag is not set, samples default to 1000.
   * `-o file`: Store the resulting plan in the given file. Two formats are possible (chosen automatically from file extension):
                  If it ends with `.csv`, stores the results by sampling each path (see flag `-s`), and put them in a CSV with samples+1 lines
                  (data+header), the header being
                  `time,[X_{ac_id},Y_{ac_id},Z_{ac_id},theta_{ac_id}]*{number of aircraft}`

                  Otherwise, stores the results in JSON format as follow (two sub-objects are defined further down):
                  {
                     <!-- "separation": (Float Number: separation value), -->
                     <!-- "z_alpha"   : (Float Number: Z distance modifier value), -->
                     "wind_x"    : (Float Number: wind_x value),
                     "wind_y"    : (Float Number: wind_y value),
                     "duration"  : (Float Number: time duration of the plan)
                     "AC_num"    : (Int Number  : number of aircraft),
                     "trajectories": [
                        {
                           "stats": {
                              "id"         : (Int Number  : aircraft id),
                              "airspeed"   : (Float Number: aircraft speed in air referential),
                              "climb"      : (Float Number: aircraft climb rate),
                              "turn_radius": (Float Number: aircraft minimal turn radius),
                           },
                           "path": {
                              "total_length"  : (Float Number : total length of the path),
                              "sections_count": (Int Number   : number of basic sectionsin this path),
                              "start"         : (Pose3D object: starting pose),
                              "end"           : (Pose3D object: ending pose),
                              "sections": [
                                 BasicPath objects,...
                              ]
                           }
                        },...
                     ]
                  }

                  A Pose3D object is defined as follow:
                  {
                     "x"    : (Float Number: X coordinate),
                     "y"    : (Float Number: Y coordinate),
                     "z"    : (Float Number: Z coordinate),
                     "theta": (Float Number: XY orientation, in radian)
                  }

                  A BasicPath object is defined as follow:
                  {
                     "length": (Float Number: Length of this section)
                     "type"  : (String      : Either "LEFT","RIGHT" or "STRAIGHT", for describing a turn or a straight line movement),
                     "x"     : (Float Number: X coordinate of the refence point; For a Straight: Starting point | For a turn: Circle center)
                     "y"     : (Float Number: Y coordinate of the refence point)
                     "z"     : (Float Number: Z coordinate of the refence point)
                     "p1"    : (Float Number: For a Straight: horizontal x speed | For a turn: Radius)
                     "p2"    : (Float Number: For a Straight: horizontal y speed | For a turn: signed angular speed; positive when LEFT, negative when RIGHT)
                     "p3"    : (Float Number: Vertical speed)
                     "p4"    : (Float Number: For a Straight: unused | For a turn: initial angle)
                  }
