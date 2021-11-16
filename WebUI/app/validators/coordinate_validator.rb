class CoordinateValidator < ActiveModel::Validator
  def validate(record)
    unless record.x.present? or record.y.present? or record.z.present?
      record.errors.add :base, "At least one of x, y, or z must be set"
    end
  end
end