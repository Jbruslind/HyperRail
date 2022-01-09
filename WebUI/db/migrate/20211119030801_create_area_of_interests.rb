class CreateAreaOfInterests < ActiveRecord::Migration[6.1]
  def change
    create_table :area_of_interests do |t|
      t.integer :analysis_id, null: false
      t.string :rectangle_id, null: false
      t.numeric :x, null: false
      t.numeric :y, null: false
      t.numeric :width, null: false
      t.numeric :height, null: false
    end

    add_foreign_key :area_of_interests, :analyses, on_delete: :cascade

    add_index :area_of_interests, :analysis_id
  end
end
