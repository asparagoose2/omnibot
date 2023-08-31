import {useSelector} from "react-redux";

const useSelectTasks = () => {
	return useSelector(state => state.tasks);
}

export default useSelectTasks;
