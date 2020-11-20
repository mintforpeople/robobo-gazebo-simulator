# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

# /*******************************************************************************
#  *
#  *   Copyright 2019, Manufactura de Ingenios Tecnológicos S.L. 
#  *   <http://www.mintforpeople.com>
#  *
#  *   Redistribution, modification and use of this software are permitted under
#  *   terms of the Apache 2.0 License.
#  *
#  *   This software is distributed in the hope that it will be useful,
#  *   but WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND; without even the implied
#  *   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  *   Apache 2.0 License for more details.
#  *
#  *   You should have received a copy of the Apache 2.0 License along with    
#  *   this software. If not, see <http://www.apache.org/licenses/>.
#  *
#  ******************************************************************************/

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
	packages=['robobo'],
	package_dir={'': 'nodes/src'},
)
setup(**setup_args)

