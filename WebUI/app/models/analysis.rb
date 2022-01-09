class Analysis < ApplicationRecord
  validates :name, presence: true

  has_many :areas_of_interest, class_name: 'AreaOfInterest'
end
