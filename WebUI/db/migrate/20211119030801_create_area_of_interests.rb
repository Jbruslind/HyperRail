class CreateAreaOfInterests < ActiveRecord::Migration[6.1]
  def change
    create_table :area_of_interests do |t|
      t.integer :analysis_id, null: false
      t.integer :x, null: false
      t.integer :y, null: false
      t.integer :width, null: false
      t.integer :height, null: false
    end

    add_foreign_key :area_of_interests, :analyses, on_delete: :cascade

    add_index :area_of_interests, :analysis_id
  end
end
