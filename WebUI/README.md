# HyperRail Web Interface

## Ruby Version

This project is configured to use ruby version 2.7.4, but that is not a hard requirement.
Changing ruby versions will require a new Gemfile and modification to `.ruby-version`.

## Configuration

Once ruby 2.7.4 is installed, the rest of the gems can be installed using `bundle install`.

## Database Setup

The SQLite database can be initialized using `bundle exec rake db:create`.
The database can be initialized using `bundle exec rake db:migrate`.
Some test data can be populated in the database by typing `bundle exec rake db:seed`.
At any point, the database can be recreated and repopulated with seed data by typing `bundle exec rake db:reset`.

The default password is `default-password` but it can be changed by creating a `Setting` in the console.
To access the console, first type `bin/rails c` and then update a `Setting` using the following example:

```ruby
# Find the existing setting or create a new one if it has not yet been created
setting = Setting.find_or_initialize_by(name: 'password')
setting.value = 'new-password-here'
setting.save!
```

## Running

To run the web server, type `bin/rails s`. By default it will run on port 3000 and can be reached at http://localhost:3000/.
Sessions are handled by storing an encrypted, signed key with the last password used and login time, and do not currently time out.
If the password setting changes, a re-login will be required as it is checked on every request.

## Testing

Currently generally unimplemented, but tests can be run using `bundle exec rake`.

## Development Notes:
Below are some helpful links to consider when modifying the interactive grid.

SVG Basics - https://developer.mozilla.org/en-US/docs/Web/SVG
How to properly scale svg elements - https://stackoverflow.com/questions/19484707/how-can-i-make-an-svg-scale-with-its-parent-container
SVG viewport and viewbox concepts - https://webdesign.tutsplus.com/tutorials/svg-viewport-and-viewbox-for-beginners--cms-30844

