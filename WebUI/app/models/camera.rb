class Camera
  attr_reader :height_m,  # meters
              :fov,     # degrees, like 47º

  def self.from_settings
    new(height_m: Setting.fetch('camera_height').to_f, fov: Setting.fetch('camera_fov').to_f)
  end

  def initialize(height_m: 1.75, fov: 47)
    @height_m, @fov = height_m, fov
  end

  # Return the x captured by the camera in meters
  def width_m
    Math.tan((fov / 2.0) * (Math::PI / 180)) * height_m * 2
  end

  def unit_sizes(rail_x: 20, rail_y: 5, crop: 25)
    # Size of image in meters after crop
    sizes = {unit_size: width_m * ((100 - crop) / 100.0)}

    # The number of units across in the x/y directions, rounding up to account for overage
    sizes[:x_units] = (rail_x / sizes[:unit_size]).ceil
    sizes[:x_unit_size] = rail_x / sizes[:x_units].to_f
    sizes[:y_units] = (rail_y / sizes[:unit_size]).ceil
    sizes[:y_unit_size] = rail_y / sizes[:y_units].to_f

    sizes
  end

  # Given a camera, return a series of x/y coordinates given the greenhouse dimensions in meters
  def to_centroids(rail_x: 20, rail_y: 5, crop: 25)
    sizes = unit_sizes(rail_x: rail_x, rail_y: rail_y, crop: crop)

    # An array of arrays
    rows = sizes[:x_units].times.map do |x|
      sizes[:y_units].times.map do |y|
        # The X and Y units, rounded and offset to be centroids
        [
          (x*sizes[:x_unit_size] + sizes[:x_unit_size]/2).round(3),
          (y*sizes[:y_unit_size] + sizes[:y_unit_size]/2).round(3)
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
