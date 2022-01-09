class AreaOfInterest < ApplicationRecord
  belongs_to :analysis

  validates :rectangle_id, presence: true
  validates :x, :y, :width, :height, presence: true, numericality: true
  validates :width, :height, numericality: { greater_than: 0 }
end
