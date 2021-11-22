class Camera
  attr_reader :height_m,  # meters
              :fov,     # degrees, like 47º

  def self.from_settings
    new(height_m: Setting.fetch('camera_height'), fov: Setting.fetch('camera_fov'))
  end

  def initialize(height_m: 1.75, fov: 47)
    @height_m, @fov = height_m, fov
  end

  # Return the x captured by the camera in meters
  def width_m
    Math.tan((fov / 2.0) * (Math::PI / 180)) * height_m * 2
  end

  # Given a camera, return a series of x/y coordinates given the greenhouse dimensions in meters
  def to_centroids(rail_x: 20, rail_y: 5, crop: 25)
    # Size of image in meters after crop
    unit_size = width_m * ((100 - crop) / 100.0)

    # The number of units across in the x/y directions, rounding up to account for overage
    x_units = (rail_x / unit_size).ceil
    x_unit_size = rail_x / x_units.to_f
    y_units = (rail_y / unit_size).ceil
    y_unit_size = rail_y / y_units.to_f

    # An array of arrays
    rows = x_units.times.map do |x|
      y_units.times.map do |y|
        # The X and Y units, rounded and offset to be centroids
        [
          (x*x_unit_size + x_unit_size/2).round(3),
          (y*y_unit_size + y_unit_size/2).round(3)
        ]
      end
    end

    # Reverse every other array to make an S pattern to minimize extra travel
    rows.each.with_index do |row, i|
      next if i.even?

      rows[i] = row.reverse
    end

    # Return the rows
    rows
  end
end
