class CreateWaypoints < ActiveRecord::Migration[6.1]
  def change
    create_table :waypoints do |t|
      t.integer :program_id, null: false
      t.integer :position, null: false
      t.numeric :x
      t.numeric :y
      t.numeric :z
      t.string :actions
    end

    add_foreign_key :waypoints, :programs, on_delete: :cascade
    add_index :waypoints, [:program_id, :position], unique: true
  end
end
